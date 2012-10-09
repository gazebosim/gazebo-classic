/*
 * Copyright 2011 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#define BOOST_FILESYSTEM_VERSION 2

#include <tinyxml.h>
#include <libtar.h>
#include <curl/curl.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "common/SystemPaths.hh"
#include "common/Console.hh"
#include "common/ModelDatabase.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
  size_t written;
  written = fwrite(ptr, size, nmemb, stream);
  return written;
}

/////////////////////////////////////////////////
size_t get_models_cb(void *_buffer, size_t _size, size_t _nmemb, void *_userp)
{
  std::string *str = static_cast<std::string*>(_userp);
  _size *= _nmemb;

  // Append the new character data to the string
  str->append(static_cast<const char*>(_buffer), _size);
  return _size;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetURI()
{
  std::string result;
  char *uriStr = getenv("GAZEBO_MODEL_DATABASE_URI");
  if (uriStr)
    result = uriStr;
  else
    gzerr << "GAZEBO_MODEL_DATABASE_URI not set\n";

  return result;
}

/////////////////////////////////////////////////
bool ModelDatabase::HasModel(const std::string &_modelURI)
{

  std::string xmlString = ModelDatabase::GetManifest();
  if (!xmlString.empty())
  {
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(xmlString.c_str());

    TiXmlElement *modelsElem = xmlDoc.FirstChildElement("models");
    TiXmlElement *modelElem;
    for (modelElem = modelsElem->FirstChildElement("model"); modelElem != NULL;
         modelElem = modelElem->NextSiblingElement("model"))
    {
      TiXmlElement *uriElem = modelElem->FirstChildElement("uri");
      std::string uri = uriElem->GetText();
      uri = "model:/" + uri;

      std::cout << "Compare[" << _modelURI << "][" << uri << "]\n";
      if (uri == _modelURI)
        return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetManifest()
{
  std::string xmlString;

  std::string uriStr = ModelDatabase::GetURI();
  if (!uriStr.empty())
  {
    std::string manifestURI = uriStr + "/manifest.xml";

    CURL *curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, manifestURI.c_str());

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, get_models_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &xmlString);

    CURLcode success = curl_easy_perform(curl);
    if (success != CURLE_OK)
    {
      gzwarn << "Unable to connect to model database using [" << uriStr
        << "]. Only locally installed models will be available.";
    }

    curl_easy_cleanup(curl);
  }

  return xmlString;
}

/////////////////////////////////////////////////
std::map<std::string, std::string> ModelDatabase::GetModels()
{
  std::map<std::string, std::string> result;
  std::string xmlString = ModelDatabase::GetManifest();
  std::string uriStr = ModelDatabase::GetURI();

  if (!xmlString.empty())
  {
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(xmlString.c_str());

    TiXmlElement *modelsElem = xmlDoc.FirstChildElement("models");
    TiXmlElement *modelElem;
    for (modelElem = modelsElem->FirstChildElement("model"); modelElem != NULL;
         modelElem = modelElem->NextSiblingElement("model"))
    {
      TiXmlElement *nameElem = modelElem->FirstChildElement("name");
      TiXmlElement *uriElem = modelElem->FirstChildElement("uri");
      result[uriStr + "/" + uriElem->GetText()] =  nameElem->GetText();
    }
  }

  return result;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelPath(const std::string &_uri)
{
  std::string path = SystemPaths::Instance()->FindFileURI(_uri);
  struct stat st;

  if (path.empty() || stat(path.c_str(), &st) != 0 )
  {
    if (!ModelDatabase::HasModel(_uri))
    {
      gzerr << "Unable to download model[" << _uri << "]\n";
      return std::string();
    }

    // DEBUG output
    std::cout << "Getting uri[" << _uri << "] path[" << path << "]\n";

    // Get the model name from the uri
    int index = _uri.find_last_of("/");
    std::string modelName = _uri.substr(index+1, _uri.size() - index - 1);

    // store zip file in temp location
    std::string filename = "/tmp/gz_model.tar.gz";

    CURL *curl = curl_easy_init();
    if (!curl)
    {
      gzerr << "Unable to initialize libcurl\n";
      return std::string();
    }

    FILE *fp = fopen(filename.c_str(), "wb");
    if (!fp)
    {
      gzerr << "Could not download model[" << _uri << "] because we were"
            << "unable to write to file[" << filename << "]."
            << "Please fix file permissions.";
      return std::string();
    }

    curl_easy_setopt(curl, CURLOPT_URL,
        (ModelDatabase::GetURI() + "/" + modelName + "/model.tar.gz").c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    CURLcode success = curl_easy_perform(curl);

    if (success != CURLE_OK)
    {
      gzerr << "Unable to connect to model database using [" << _uri << "]\n";
      return std::string();
    }

    curl_easy_cleanup(curl);

    fclose(fp);

    // Unzip model tarball
    std::ifstream file(filename.c_str(),
        std::ios_base::in | std::ios_base::binary);
    std::ofstream out("/tmp/gz_model.tar",
        std::ios_base::out | std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(file);
    boost::iostreams::copy(in, out);

    TAR *tar;
    tar_open(&tar, const_cast<char*>("/tmp/gz_model.tar"),
             NULL, O_RDONLY, 0644, TAR_GNU);

    std::string outputPath = getenv("HOME");
    outputPath += "/.gazebo/models";

    tar_extract_all(tar, const_cast<char*>(outputPath.c_str()));
    path = outputPath + "/" + modelName;

    ModelDatabase::DownloadDependencies(path);
  }

  return path;
}

/////////////////////////////////////////////////
void ModelDatabase::DownloadDependencies(const std::string &_path)
{
  std::string manifest = _path + "/manifest.xml";

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(manifest))
  {
    TiXmlElement *dependXML = xmlDoc.FirstChildElement("depend");
    if (!dependXML)
      return;

    for(TiXmlElement *modelXML = dependXML->FirstChildElement("model");
        modelXML; modelXML = modelXML->NextSiblingElement())
    {
      TiXmlElement *uriXML = modelXML->FirstChildElement("uri");
      if (uriXML)
      {
        // Download the model if it doesn't exist.
        ModelDatabase::GetModelPath(uriXML->GetText());
      }
      else
      {
        gzerr << "Model depend is missing <uri> in manifest["
              << manifest << "]\n";
      }
    }
  }
  else
    gzerr << "Unable to load manifest file[" << manifest << "]\n";
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelFile(const std::string &_uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = ModelDatabase::GetModelPath(_uri);

  std::string manifest = path + "/manifest.xml";

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(manifest))
  {
    TiXmlElement *modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
      if (sdfXML)
      {
        result = path + "/" + sdfXML->GetText();
      }
      else
      {
        gzerr << "Manifest[" << manifest << "] doesn't have "
              << "<model><sdf>...</sdf></model> element.\n";
      }
    }
    else
    {
      gzerr << "Manifest[" << manifest << "] doesn't have a <model> element\n";
    }
  }
  else
  {
    gzerr << "Invalid model manifest file[" << manifest << "]\n";
  }

  return result;
}
