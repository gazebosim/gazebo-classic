/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <tinyxml.h>
#include <libtar.h>
#include <curl/curl.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "gazebo/sdf/sdf.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/ModelDatabase.hh"

using namespace gazebo;
using namespace common;

ModelDatabase *ModelDatabase::myself = ModelDatabase::Instance();

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
ModelDatabase::ModelDatabase()
{
  this->stop = false;

  // Create the thread that is used to update the model cache. This
  // retreives online data in the background to improve startup times.
  this->updateCacheThread = new boost::thread(
      boost::bind(&ModelDatabase::UpdateModelCache, this));
}

/////////////////////////////////////////////////
ModelDatabase::~ModelDatabase()
{
  // Stop the update thread.
  this->stop = true;
  this->updateCacheCondition.notify_one();
  this->updateCacheThread->join();

  delete this->updateCacheThread;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetURI()
{
  std::string result;
  char *uriStr = getenv("GAZEBO_MODEL_DATABASE_URI");
  if (uriStr)
    result = uriStr;
  else
    gzwarn << "GAZEBO_MODEL_DATABASE_URI not set\n";

  if (result[result.size()-1] != '/')
    result += '/';

  return result;
}

/////////////////////////////////////////////////
bool ModelDatabase::HasModel(const std::string &_modelURI)
{
  std::string uri = _modelURI;
  boost::replace_first(uri, "model://", ModelDatabase::GetURI());

  std::map<std::string, std::string> models = ModelDatabase::GetModels();
  for (std::map<std::string, std::string>::iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    if (iter->first == uri)
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetManifest(const std::string &_uri)
{
  std::string xmlString;
  std::string uri = _uri;
  boost::replace_first(uri, "model://", ModelDatabase::GetURI());

  if (!uri.empty())
  {
    std::string manifestURI = uri + "/manifest.xml";

    CURL *curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, manifestURI.c_str());

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, get_models_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &xmlString);

    CURLcode success = curl_easy_perform(curl);
    if (success != CURLE_OK)
    {
      gzwarn << "Unable to connect to model database using [" << _uri
        << "]. Only locally installed models will be available.\n";
    }

    curl_easy_cleanup(curl);
  }

  return xmlString;
}


/////////////////////////////////////////////////
bool ModelDatabase::UpdateModelCacheImpl()
{
  std::string xmlString = ModelDatabase::GetManifest(ModelDatabase::GetURI());

  if (!xmlString.empty())
  {
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(xmlString.c_str());

    TiXmlElement *databaseElem = xmlDoc.FirstChildElement("database");
    if (!databaseElem)
    {
      gzerr << "No <database> tag in the model database manifest.xml found"
        << " here[" << ModelDatabase::GetURI() << "]\n";
      return false;
    }

    TiXmlElement *modelsElem = databaseElem->FirstChildElement("models");
    if (!modelsElem)
    {
      gzerr << "No <models> tag in the model database manifest.xml found"
        << " here[" << ModelDatabase::GetURI() << "]\n";
      return false;
    }

    TiXmlElement *uriElem;
    for (uriElem = modelsElem->FirstChildElement("uri"); uriElem != NULL;
        uriElem = uriElem->NextSiblingElement("uri"))
    {
      std::string uri = uriElem->GetText();

      size_t index = uri.find("://");
      std::string suffix = uri;
      if (index != std::string::npos)
      {
        suffix = uri.substr(index + 3, uri.size() - index - 3);
      }

      std::string fullURI = ModelDatabase::GetURI() + suffix;
      std::string modelName = ModelDatabase::GetModelName(fullURI);

      this->modelCache[fullURI] = modelName;
    }
  }

  return true;
}

/////////////////////////////////////////////////
void ModelDatabase::UpdateModelCache()
{
  // Continually update the model cache when requested.
  while (!this->stop)
  {
    // Wait for an update request.
    boost::mutex::scoped_lock lock(this->updateMutex);
    this->updateCacheCondition.wait(lock);

    // Exit if notified and stopped.
    if (this->stop)
      break;

    // Update the model cache.
    if (!this->UpdateModelCacheImpl())
      gzerr << "Unable to download model manifests\n";
    else
    {
      for (std::list<CallbackFunc>::iterator iter = this->callbacks.begin();
           iter != this->callbacks.end(); ++iter)
      {
        (*iter)(this->modelCache);
      }
      this->callbacks.clear();
    }
  }
}

/////////////////////////////////////////////////
std::map<std::string, std::string> ModelDatabase::GetModels()
{
  size_t size = 0;

  {
    boost::mutex::scoped_lock lock(this->updateMutex);
    size = this->modelCache.size();
  }

  if (size != 0)
    return this->modelCache;
  else
  {
    gzwarn << "Getting models from[" << GetURI()
           << "]. This may take a few seconds.\n";

    // Tell the background thread to grab the models from online.
    this->updateCacheCondition.notify_one();

    // Let the other thread start downloading.
    common::Time::MSleep(100);

    // Wait for the thread to finish.
    boost::mutex::scoped_lock lock(this->updateMutex);
  }

  return this->modelCache;
}

/////////////////////////////////////////////////
void ModelDatabase::GetModels(
    boost::function<void (const std::map<std::string, std::string> &)> _func)
{
  this->callbacks.push_back(_func);
  this->updateCacheCondition.notify_one();
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelName(const std::string &_uri)
{
  std::string result;
  std::string xmlStr = ModelDatabase::GetManifest(_uri);

  if (!xmlStr.empty())
  {
    TiXmlDocument xmlDoc;
    if (xmlDoc.Parse(xmlStr.c_str()))
    {
      TiXmlElement *modelElem = xmlDoc.FirstChildElement("model");
      if (modelElem)
      {
        TiXmlElement *nameElem = modelElem->FirstChildElement("name");
        if (nameElem)
          result = nameElem->GetText();
        else
          gzerr << "No <name> element in manifest.xml for model["
                << _uri << "]\n";
      }
      else
        gzerr << "No <model> element in manifest.xml for model["
              << _uri << "]\n";
    }
    else
      gzerr << "Unable to parse manifest.xml for model[" << _uri << "]\n";
  }
  else
    gzerr << "Unable to get model name[" << _uri << "]\n";

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
    // std::cout << "Getting uri[" << _uri << "] path[" << path << "]\n";

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
    curl_easy_setopt(curl, CURLOPT_URL,
        (ModelDatabase::GetURI() + "/" +
         modelName + "/model.tar.gz").c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);

    bool retry = true;
    int iterations = 0;
    while (retry && iterations < 4)
    {
      retry = false;
      iterations++;

      FILE *fp = fopen(filename.c_str(), "wb");
      if (!fp)
      {
        gzerr << "Could not download model[" << _uri << "] because we were"
          << "unable to write to file[" << filename << "]."
          << "Please fix file permissions.";
        return std::string();
      }

      /// Download the model tarball
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
      CURLcode success = curl_easy_perform(curl);

      if (success != CURLE_OK)
      {
        gzwarn << "Unable to connect to model database using ["
               << _uri << "]\n";
        retry = true;
        continue;
      }

      fclose(fp);

      try
      {
        // Unzip model tarball
        std::ifstream file(filename.c_str(),
            std::ios_base::in | std::ios_base::binary);
        std::ofstream out("/tmp/gz_model.tar",
            std::ios_base::out | std::ios_base::binary);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(file);
        boost::iostreams::copy(in, out);
      }
      catch(...)
      {
        gzerr << "Failed to unzip model tarball. Trying again...\n";
        retry = true;
        continue;
      }

      TAR *tar;
      tar_open(&tar, const_cast<char*>("/tmp/gz_model.tar"),
          NULL, O_RDONLY, 0644, TAR_GNU);

      std::string outputPath = getenv("HOME");
      outputPath += "/.gazebo/models";

      tar_extract_all(tar, const_cast<char*>(outputPath.c_str()));
      path = outputPath + "/" + modelName;

      ModelDatabase::DownloadDependencies(path);
    }

    curl_easy_cleanup(curl);
    if (retry)
    {
      gzerr << "Could not download model[" << _uri << "]."
        << "The model may be corrupt.\n";
      path.clear();
    }
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
    TiXmlElement *modelXML = xmlDoc.FirstChildElement("model");
    if (!modelXML)
    {
      gzerr << "No <model> element in manifest file[" << _path << "]\n";
      return;
    }

    TiXmlElement *dependXML = modelXML->FirstChildElement("depend");
    if (!dependXML)
      return;

    for (TiXmlElement *depXML = dependXML->FirstChildElement("model");
         depXML; depXML = depXML->NextSiblingElement())
    {
      TiXmlElement *uriXML = depXML->FirstChildElement("uri");
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
      TiXmlElement *sdfSearch = sdfXML;

      // Find the SDF element that matches our current SDF version.
      while (sdfSearch)
      {
        if (sdfSearch->Attribute("version") &&
            std::string(sdfSearch->Attribute("version")) == SDF_VERSION)
        {
          sdfXML = sdfSearch;
          break;
        }

        sdfSearch = sdfSearch->NextSiblingElement("sdf");
      }

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
