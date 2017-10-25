/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <ignition/fuel-tools.hh>

#include <sdf/sdf.hh>

#include "gazebo/common/Time.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/FuelModelDatabase.hh"
#include "gazebo/common/SemanticVersion.hh"

using namespace gazebo;
using namespace common;

FuelModelDatabase *FuelModelDatabase::myself = FuelModelDatabase::Instance();

namespace gazebo
{
namespace common
{
/// \brief Private class attributes for FuelModelDatabase.
class FuelModelDatabasePrivate
{
  /// \brief Thread to update the model cache.
  public: boost::thread *updateCacheThread;

  /// \brief A dictionary of all model names indexed by their uri.
  public: std::map<std::string, std::string> modelCache;

  /// \brief True to stop the background thread
  public: bool stop;

  /// \brief Cache update mutex.
  public: boost::mutex updateMutex;

  /// \brief Protects callback list.
  public: boost::mutex callbacksMutex;

  /// \brief Mutex to protect cache thread status checks.
  public: boost::recursive_mutex startCacheMutex;

  /// \brief Condition variable for the updateCacheThread.
  public: boost::condition_variable updateCacheCondition;

  /// \brief Condition variable for completion of one cache update.
  public: boost::condition_variable updateCacheCompleteCondition;

  /// \def CallbackFunc
  /// \brief Boost function that is used to passback the model cache.
  public: typedef boost::function<
           void (const std::map<std::string, std::string> &)> CallbackFunc;

  /// \brief Triggered when the model data has been updated after
  /// calling ModelDatabase::GetModels()
  public: event::EventT<
            void (std::map<std::string, std::string>)> modelDBUpdated;

  /// \brief ToDo.
  public: std::unique_ptr<ignition::fuel_tools::FuelClient> client;
};
}
}

/////////////////////////////////////////////////
FuelModelDatabase::FuelModelDatabase()
  : dataPtr(new FuelModelDatabasePrivate)
{
  this->dataPtr->updateCacheThread = nullptr;
  this->Start();

  // Create a ClientConfig, TODO create this from a yaml file.
  ignition::fuel_tools::ClientConfig conf;
  ignition::fuel_tools::ServerConfig srv;
  srv.URL("https://staging-api.ignitionfuel.org");
  srv.LocalName("ignitionfuel");
  conf.AddServer(srv);

  this->dataPtr->client.reset(new ignition::fuel_tools::FuelClient(conf));
}

/////////////////////////////////////////////////
FuelModelDatabase::~FuelModelDatabase()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void FuelModelDatabase::Start(bool _fetchImmediately)
{
  boost::recursive_mutex::scoped_lock lock(this->dataPtr->startCacheMutex);

  if (!this->dataPtr->updateCacheThread)
  {
    this->dataPtr->stop = false;

    // Create the thread that is used to update the model cache. This
    // retreives online data in the background to improve startup times.
    this->dataPtr->updateCacheThread = new boost::thread(
        boost::bind(
            &FuelModelDatabase::UpdateModelCache, this, _fetchImmediately));
  }
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::GetURI()
{
  std::string result;

  // begin testing
  result = "Ignition Fuel models";
  // end testing

  //char *uriStr = getenv("GAZEBO_MODEL_DATABASE_URI");
  //if (uriStr)
  //  result = uriStr;
  //else
  //{
  //  // No env var.  Take compile-time default.
  //  result = GAZEBO_MODEL_DATABASE_URI;
  //}
//
  //if (result[result.size()-1] != '/')
  //  result += '/';

  return result;
}

/////////////////////////////////////////////////
event::ConnectionPtr FuelModelDatabase::GetModels(
    boost::function<void (const std::map<std::string, std::string> &)> _func)
{
  boost::mutex::scoped_lock lock2(this->dataPtr->callbacksMutex);
  this->dataPtr->updateCacheCondition.notify_one();
  return this->dataPtr->modelDBUpdated.Connect(_func);
}

/////////////////////////////////////////////////
bool FuelModelDatabase::UpdateModelCacheImpl()
{
  //std::string xmlString = FuelModelDatabase::GetDBConfig(ModelDatabase::GetURI());
//
  //if (!xmlString.empty())
  //{
  //  TiXmlDocument xmlDoc;
  //  xmlDoc.Parse(xmlString.c_str());
//
  //  TiXmlElement *databaseElem = xmlDoc.FirstChildElement("database");
  //  if (!databaseElem)
  //  {
  //    gzerr << "No <database> tag in the model database "
  //          << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
  //          << " here[" << FuelModelDatabase::GetURI() << "]\n";
  //    return false;
  //  }
//
  //  TiXmlElement *modelsElem = databaseElem->FirstChildElement("models");
  //  if (!modelsElem)
  //  {
  //    gzerr << "No <models> tag in the model database "
  //      << GZ_MODEL_DB_MANIFEST_FILENAME << " found"
  //      << " here[" << FuelModelDatabase::GetURI() << "]\n";
  //    return false;
  //  }
//
  //  TiXmlElement *uriElem;
//
  //  for (uriElem = modelsElem->FirstChildElement("uri");
  //       uriElem != nullptr && !this->dataPtr->stop;
  //       uriElem = uriElem->NextSiblingElement("uri"))
  //  {
  //    std::string uri = uriElem->GetText();
//
  //    size_t index = uri.find("://");
  //    std::string suffix = uri;
  //    if (index != std::string::npos)
  //    {
  //      suffix = uri.substr(index + 3, uri.size() - index - 3);
  //    }
//
  //    std::string fullURI = FuelModelDatabase::GetURI() + suffix;
  //    std::string modelName = FuelModelDatabase::GetModelName(fullURI);
//
  //    this->dataPtr->modelCache[fullURI] = modelName;
  //  }
  //}

  for (auto iter = this->dataPtr->client->Models(); iter; ++iter)
  {
    std::string name = iter->Identification().Name();
    this->dataPtr->modelCache[name] = name;
  }

  //this->dataPtr->modelCache["https://xxx/model1"] = "model1";
  //this->dataPtr->modelCache["https://xxx/model2"] = "model2";
  //this->dataPtr->modelCache["https://xxx/model3"] = "model3";

  return true;
}

/////////////////////////////////////////////////
void FuelModelDatabase::UpdateModelCache(bool _fetchImmediately)
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
      gzerr << "Unable to get the list of models from Fuel\n";
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
