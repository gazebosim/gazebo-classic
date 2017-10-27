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

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <sdf/sdf.hh>
#include <ignition/fuel-tools.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/FuelModelDatabase.hh"

using namespace gazebo;
using namespace common;

/// \brief Private class attributes for FuelModelDatabase.
class gazebo::common::FuelModelDatabasePrivate
{
  /// \brief The server name.
  public: std::string server;

  /// \brief Thread to update the model cache.
  public: std::thread *updateCacheThread;

  /// \brief A dictionary of all model names indexed by their uri.
  public: std::map<std::string, std::string> modelCache;

  /// \brief True to stop the background thread
  public: bool stop;

  /// \brief Cache update mutex.
  public: std::mutex updateMutex;

  /// \brief Protects callback list.
  public: std::mutex callbacksMutex;

  /// \brief Mutex to protect cache thread status checks.
  public: std::recursive_mutex startCacheMutex;

  /// \brief Condition variable for the updateCacheThread.
  public: std::condition_variable updateCacheCondition;

  /// \brief Condition variable for completion of one cache update.
  public: std::condition_variable updateCacheCompleteCondition;

  /// \brief Triggered when the model data has been updated after
  /// calling FuelModelDatabase::Models().
  public: event::EventT<
            void (std::map<std::string, std::string>)> modelDBUpdated;

  /// \brief A client to interact with Ignition Fuel.
  public: std::unique_ptr<ignition::fuel_tools::FuelClient> fuelClient;
};

/////////////////////////////////////////////////
FuelModelDatabase::FuelModelDatabase(const std::string &_server)
  : dataPtr(new FuelModelDatabasePrivate)
{
  this->dataPtr->server = _server;
  this->dataPtr->updateCacheThread = nullptr;

  // Create a ClientConfig, TODO create this from a yaml file.
  ignition::fuel_tools::ServerConfig srv;
  srv.URL(this->dataPtr->server);
  srv.LocalName("ignitionfuel");
  ignition::fuel_tools::ClientConfig conf;
  conf.AddServer(srv);

  this->dataPtr->fuelClient.reset(new ignition::fuel_tools::FuelClient(conf));

  this->Start();
}

/////////////////////////////////////////////////
FuelModelDatabase::~FuelModelDatabase()
{
}

/////////////////////////////////////////////////
void FuelModelDatabase::Start(const bool _fetchImmediately)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->startCacheMutex);

  if (!this->dataPtr->updateCacheThread)
  {
    this->dataPtr->stop = false;

    // Create the thread that is used to update the model cache. This
    // retreives online data in the background to improve startup times.
    this->dataPtr->updateCacheThread =
        new std::thread(std::bind(
            &FuelModelDatabase::UpdateModelCache, this, _fetchImmediately));
  }
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::URI() const
{
  return this->dataPtr->server;
}

/////////////////////////////////////////////////
event::ConnectionPtr FuelModelDatabase::Models(
    std::function<void (const std::map<std::string, std::string> &)> _func)
{
  std::lock_guard<std::mutex> lock2(this->dataPtr->callbacksMutex);
  this->dataPtr->updateCacheCondition.notify_one();
  return this->dataPtr->modelDBUpdated.Connect(_func);
}

/////////////////////////////////////////////////
std::map<std::string, std::string> FuelModelDatabase::Models2()
{
  std::map<std::string, std::string> models;
  for (auto iter = this->dataPtr->fuelClient->Models(); iter; ++iter)
  {
    std::string name = iter->Identification().Name();
    models[name] = name;
  }

  return models;
}

/////////////////////////////////////////////////
void FuelModelDatabase::Models2Async(
    std::function<void (const std::map<std::string, std::string> &)> &_func)
{
  std::thread t([this, &_func]()
  {
    auto models = this->Models2();
    std::cout << "Inside thread" << std::endl;
    _func(models);
  });

  t.detach();
}


/////////////////////////////////////////////////
bool FuelModelDatabase::UpdateModelCacheImpl()
{
  for (auto iter = this->dataPtr->fuelClient->Models(); iter; ++iter)
  {
    std::string name = iter->Identification().Name();
    this->dataPtr->modelCache[name] = name;
  }

  return true;
}

/////////////////////////////////////////////////
void FuelModelDatabase::UpdateModelCache(bool _fetchImmediately)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->updateMutex);

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
      std::unique_lock<std::mutex> lock2(this->dataPtr->callbacksMutex);
      if (this->dataPtr->stop)
        break;
      this->dataPtr->modelDBUpdated(this->dataPtr->modelCache);
    }
    this->dataPtr->updateCacheCompleteCondition.notify_all();
  }

  // Make sure no one is waiting on us.
  this->dataPtr->updateCacheCompleteCondition.notify_all();
}
