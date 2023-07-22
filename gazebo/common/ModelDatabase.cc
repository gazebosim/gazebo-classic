/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _WIN32
#include <libtar.h>
#endif
#include <curl/curl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <iostream>

#include <boost/algorithm/string/replace.hpp>
#include <boost/bind/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/Time.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/ModelDatabasePrivate.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/common/SemanticVersion.hh"
#include "gazebo/common/CommonIface.hh"

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
  : dataPtr(new ModelDatabasePrivate)
{
  this->dataPtr->updateCacheThread = nullptr;
  this->Start();
}

/////////////////////////////////////////////////
ModelDatabase::~ModelDatabase()
{
  this->Fini();
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void ModelDatabase::Start(bool _fetchImmediately)
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->startCacheMutex);

  if (!this->dataPtr->updateCacheThread)
  {
    this->dataPtr->stop = false;

    // Create the thread that is used to update the model cache. This
    // retreives online data in the background to improve startup times.
    this->dataPtr->updateCacheThread = new boost::thread(
        boost::bind(&ModelDatabase::UpdateModelCache, this, _fetchImmediately));
  }
}

/////////////////////////////////////////////////
void ModelDatabase::Fini()
{
  // Stop the update thread.
  this->dataPtr->stop = true;
  this->dataPtr->updateCacheCompleteCondition.notify_all();
  this->dataPtr->updateCacheCondition.notify_all();

  {
    boost::recursive_mutex::scoped_lock lock(this->dataPtr->startCacheMutex);
    if (this->dataPtr->updateCacheThread)
      this->dataPtr->updateCacheThread->join();
    delete this->dataPtr->updateCacheThread;
    this->dataPtr->updateCacheThread = nullptr;
  }
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetURI()
{
  std::string result;
  char *uriStr = getenv("GAZEBO_MODEL_DATABASE_URI");
  if (uriStr)
    result = uriStr;
  else
  {
    // No env var.  Take compile-time default.
    result = GAZEBO_MODEL_DATABASE_URI;
  }

  if (result[result.size()-1] != '/')
    result += '/';

  return result;
}

/////////////////////////////////////////////////
bool ModelDatabase::HasModel(const std::string &_modelURI)
{
  std::string uri = _modelURI;

  size_t uriSeparator = uri.find("://");

  // Make sure there is a URI separator
  if (uriSeparator == std::string::npos)
  {
    gzerr << "No URI separator \"://\" in [" << _modelURI << "]\n";
    return false;
  }

  boost::replace_first(uri, "model://", ModelDatabase::GetURI());
  uri = uri.substr(0, uri.find("/", ModelDatabase::GetURI().size()));

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
std::string ModelDatabase::GetDBConfig(const std::string &_uri)
{
  std::string xmlString;
  std::string uri = _uri;
  boost::replace_first(uri, "model://", ModelDatabase::GetURI());

  if (!uri.empty())
  {
    std::string manifestURI = uri + "/" + GZ_MODEL_DB_MANIFEST_FILENAME;
    xmlString = this->GetManifestImpl(manifestURI);
  }

  return xmlString;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelConfig(const std::string &_uri)
{
  std::string xmlString;
  std::string uri = _uri;
  boost::replace_first(uri, "model://", ModelDatabase::GetURI());

  if (!uri.empty())
  {
    std::string manifestURI = uri + "/" + GZ_MODEL_MANIFEST_FILENAME;
    xmlString = this->GetManifestImpl(manifestURI);
  }

  return xmlString;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetManifestImpl(const std::string &_uri)
{
  std::string xmlString;
  if (!_uri.empty())
  {
    CURL *curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, _uri.c_str());

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
  std::string xmlString = ModelDatabase::GetDBConfig(ModelDatabase::GetURI());

  if (!xmlString.empty())
  {
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(xmlString.c_str());

    TiXmlElement *databaseElem = xmlDoc.FirstChildElement("database");
    if (!databaseElem)
    {
      gzerr << "No <database> tag in the model database "
            << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
            << " here[" << ModelDatabase::GetURI() << "]\n";
      return false;
    }

    TiXmlElement *modelsElem = databaseElem->FirstChildElement("models");
    if (!modelsElem)
    {
      gzerr << "No <models> tag in the model database "
        << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
        << " here[" << ModelDatabase::GetURI() << "]\n";
      return false;
    }

    TiXmlElement *uriElem;
    for (uriElem = modelsElem->FirstChildElement("uri");
         uriElem != nullptr && !this->dataPtr->stop;
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

      this->dataPtr->modelCache[fullURI] = modelName;
    }
  }

  return true;
}

/////////////////////////////////////////////////
void ModelDatabase::UpdateModelCache(bool _fetchImmediately)
{
  boost::mutex::scoped_lock lock(this->dataPtr->updateMutex);

  // Continually update the model cache when requested.
  while (!this->dataPtr->stop)
  {
    // Wait for an update request.
    if (!_fetchImmediately)
      this->dataPtr->updateCacheCondition.wait(lock);
    else
      _fetchImmediately = false;

    // Exit if notified and stopped.
    if (this->dataPtr->stop)
      break;

    // Update the model cache.
    if (!this->UpdateModelCacheImpl())
      gzerr << "Unable to download model manifests\n";
    else
    {
      boost::mutex::scoped_lock lock2(this->dataPtr->callbacksMutex);
      if (this->dataPtr->stop)
        break;
      this->dataPtr->modelDBUpdated(this->dataPtr->modelCache);
    }
    this->dataPtr->updateCacheCompleteCondition.notify_all();
  }

  // Make sure no one is waiting on us.
  this->dataPtr->updateCacheCompleteCondition.notify_all();
}

/////////////////////////////////////////////////
std::map<std::string, std::string> ModelDatabase::GetModels()
{
  size_t size = 0;

  {
    boost::recursive_mutex::scoped_lock startLock(
        this->dataPtr->startCacheMutex);
    if (!this->dataPtr->updateCacheThread)
    {
      boost::mutex::scoped_lock lock(this->dataPtr->updateMutex);
      this->Start(true);
      this->dataPtr->updateCacheCompleteCondition.wait(lock);
    }
    else
    {
      boost::mutex::scoped_try_lock lock(this->dataPtr->updateMutex);
      if (!lock)
      {
        gzmsg << "Waiting for model database update to complete...\n";
        boost::mutex::scoped_lock lock2(this->dataPtr->updateMutex);
      }
    }

    size = this->dataPtr->modelCache.size();
  }

  if (size != 0)
    return this->dataPtr->modelCache;
  else
  {
    gzwarn << "Getting models from[" << GetURI()
           << "]. This may take a few seconds.\n";

    boost::mutex::scoped_lock lock(this->dataPtr->updateMutex);

    // Tell the background thread to grab the models from online.
    this->dataPtr->updateCacheCondition.notify_all();

    // Wait for the thread to finish.
    this->dataPtr->updateCacheCompleteCondition.wait(lock);
  }

  return this->dataPtr->modelCache;
}

/////////////////////////////////////////////////
event::ConnectionPtr ModelDatabase::GetModels(
    boost::function<void (const std::map<std::string, std::string> &)> _func)
{
  boost::mutex::scoped_lock lock2(this->dataPtr->callbacksMutex);
  this->dataPtr->updateCacheCondition.notify_one();
  return this->dataPtr->modelDBUpdated.Connect(_func);
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelName(const std::string &_uri)
{
  std::string result;
  std::string xmlStr = ModelDatabase::GetModelConfig(_uri);

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
          gzerr << "No <name> element in " << GZ_MODEL_MANIFEST_FILENAME
                << " for model[" << _uri << "]\n";
      }
      else
        gzerr << "No <model> element in " << GZ_MODEL_MANIFEST_FILENAME
              << " for model[" << _uri << "]\n";
    }
    else
      gzerr << "Unable to parse " << GZ_MODEL_MANIFEST_FILENAME
            << " for model[" << _uri << "]\n";
  }
  else
    gzerr << "Unable to get model name[" << _uri << "]\n";

  return result;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelPath(const std::string &_uri,
                                        bool _forceDownload)
{
  std::string path, suffix;

  if (!_forceDownload)
    path = SystemPaths::Instance()->FindFileURI(_uri);

  struct stat st;

  if (path.empty() || stat(path.c_str(), &st) != 0 )
  {
    if (!ModelDatabase::HasModel(_uri))
    {
      return std::string();
    }

    // Get the model name from the uri
    size_t startIndex = _uri.find_first_of("://");
    if (startIndex == std::string::npos)
    {
      gzerr << "URI[" << _uri << "] is missing ://\n";
      return std::string();
    }

    std::string modelName = _uri;
    boost::replace_first(modelName, "model://", "");
    boost::replace_first(modelName, ModelDatabase::GetURI(), "");

    startIndex = modelName[0] == '/' ? 1 : 0;
    size_t endIndex = modelName.find_first_of("/", startIndex);
    size_t modelNameLen = endIndex == std::string::npos ? std::string::npos :
      endIndex - startIndex;

    if (endIndex != std::string::npos)
      suffix = modelName.substr(endIndex, std::string::npos);

    modelName = modelName.substr(startIndex, modelNameLen);

    // Store downloaded .tar.gz and intermediate .tar files in temp location
    boost::filesystem::path tmppath = boost::filesystem::temp_directory_path();
    tmppath /= boost::filesystem::unique_path("gz_model-%%%%-%%%%-%%%%-%%%%");
    std::string tarfilename = tmppath.string() + ".tar";
    std::string tgzfilename = tarfilename + ".gz";

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

      FILE *fp = fopen(tgzfilename.c_str(), "wb");
      if (!fp)
      {
        gzerr << "Could not download model[" << _uri << "] because we were"
          << "unable to write to file[" << tgzfilename << "]."
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
        std::ifstream file(tgzfilename.c_str(),
            std::ios_base::in | std::ios_base::binary);
        std::ofstream out(tarfilename.c_str(),
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

      std::string outputPath = getenv(HOMEDIR);
      outputPath += "/.gazebo/models";

#ifndef _WIN32
      TAR *tar;
      tar_open(&tar, const_cast<char*>(tarfilename.c_str()),
          nullptr, O_RDONLY, 0644, TAR_GNU);

      tar_extract_all(tar, const_cast<char*>(outputPath.c_str()));
#else
      // Tar now is a built-in tool since Windows 10 build 17063.
      std::string cmdline = "tar xzf \"";
      cmdline += tarfilename + "\" -C \"";
      cmdline += outputPath + "\"";
      auto ret = system(cmdline.c_str());
      if (ret != 0)
      {
        gzerr << "tar extract ret = " << ret << ", cmdline = " << cmdline
              << std::endl;
      }
#endif
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

    // Clean up
    try
    {
      boost::filesystem::remove(tarfilename);
      boost::filesystem::remove(tgzfilename);
    }
    catch(...)
    {
      gzwarn << "Failed to remove temporary model files after download.";
    }
  }

  return path + suffix;
}

/////////////////////////////////////////////////
void ModelDatabase::DownloadDependencies(const std::string &_path)
{
  boost::filesystem::path manifestPath = _path;

  // Get the GZ_MODEL_MANIFEST_FILENAME.
  if (boost::filesystem::exists(manifestPath / GZ_MODEL_MANIFEST_FILENAME))
    manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
  else
  {
    gzerr << "Missing " << GZ_MODEL_MANIFEST_FILENAME
      << " for model " << _path << "\n";
  }

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(manifestPath.string()))
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
              << manifestPath << "]\n";
      }
    }
  }
  else
    gzerr << "Unable to load manifest file[" << manifestPath << "]\n";
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelFile(const std::string &_uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = ModelDatabase::GetModelPath(_uri);

  boost::filesystem::path manifestPath = path;

  // Get the GZ_MODEL_MANIFEST_FILENAME.
  if (boost::filesystem::exists(manifestPath / GZ_MODEL_MANIFEST_FILENAME))
    manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
  else
  {
    gzerr << "Missing " << GZ_MODEL_MANIFEST_FILENAME
      << " for model " << manifestPath << "\n";
  }

  TiXmlDocument xmlDoc;
  SemanticVersion sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";
  if (xmlDoc.LoadFile(manifestPath.string()))
  {
    TiXmlElement *modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
      TiXmlElement *sdfSearch = sdfXML;

      // Find the SDF element that matches our current SDF version.
      // If a match is not found, use the latest version of the element
      // that is not older than the SDF parser.
      while (sdfSearch)
      {
        if (sdfSearch->Attribute("version"))
        {
          std::string version = std::string(sdfSearch->Attribute("version"));
          SemanticVersion modelVersion(version);
          SemanticVersion bestVersion(bestVersionStr);
          if (modelVersion > bestVersion)
          {
            // this model is better than the previous one
            if (modelVersion <= sdfParserVersion)
            {
              // the parser can read it
              sdfXML = sdfSearch;
              bestVersionStr = version;
            }
            else
            {
              gzwarn << "Ignoring version " << version
                << " for model " << _uri
                << " because Gazebo is using an older sdf parser (version "
                << SDF_VERSION << ")" << std::endl;
            }
          }
        }
        sdfSearch = sdfSearch->NextSiblingElement("sdf");
      }

      if (sdfXML)
      {
        result = path + "/" + sdfXML->GetText();
      }
      else
      {
        gzerr << "Manifest[" << manifestPath << "] doesn't have "
              << "<model><sdf>...</sdf></model> element.\n";
      }
    }
    else
    {
      gzerr << "Manifest[" << manifestPath
            << "] doesn't have a <model> element\n";
    }
  }
  else
  {
    gzerr << "Invalid model manifest file[" << manifestPath << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
ModelDatabase* ModelDatabase::Instance()
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return SingletonT<ModelDatabase>::Instance();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}
