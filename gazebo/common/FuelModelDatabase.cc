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

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <ignition/fuel-tools.hh>

#include "gazebo/common/FuelModelDatabase.hh"

using namespace gazebo;
using namespace common;

FuelModelDatabase *FuelModelDatabase::myself = FuelModelDatabase::Instance();

/// \brief Private class attributes for FuelModelDatabase.
class gazebo::common::FuelModelDatabasePrivate
{
  /// \brief A client to interact with Ignition Fuel.
  public: std::unique_ptr<ignition::fuel_tools::FuelClient> fuelClient;
};

/////////////////////////////////////////////////
FuelModelDatabase::FuelModelDatabase()
  : dataPtr(new FuelModelDatabasePrivate)
{
  // ToDo: Remove this block when Ignition Fuel Tools supports parsing
  // a configuration file.
  ignition::fuel_tools::ServerConfig srv;
  srv.URL("https://staging-api.ignitionfuel.org");
  ignition::fuel_tools::ClientConfig conf;
  conf.AddServer(srv);

  this->dataPtr->fuelClient.reset(new ignition::fuel_tools::FuelClient(conf));
}

/////////////////////////////////////////////////
FuelModelDatabase::~FuelModelDatabase()
{
}

/////////////////////////////////////////////////
std::vector<std::string> FuelModelDatabase::Servers() const
{
  auto servers = this->dataPtr->fuelClient->Config().Servers();
  std::vector<std::string> res;
  for (const auto &server : servers)
    res.push_back(server.URL());

  return res;
}

/////////////////////////////////////////////////
void FuelModelDatabase::Models(const std::string &_server,
    std::function<void (const std::map<std::string, std::string> &)> &_func)
{
  std::thread t([this, _func, _server]
  {
    // Run the callback passing the list of models.
    _func(this->Models(_server));
  });
  t.detach();
}

/////////////////////////////////////////////////
std::map<std::string, std::string> FuelModelDatabase::Models(
    const std::string &_server) const
{
  std::map<std::string, std::string> models;

  // Sanity check: Verity that the server is correct.
  auto servers = this->dataPtr->fuelClient->Config().Servers();
  bool serverFound = false;
  for (const auto &server : servers)
  {
    if (server.URL() == _server)
    {
      serverFound = true;
      break;
    }
  }
  if (!serverFound)
    return models;


  // ToDo: Pass the server name when Ignition Fuel Tools supports multiple
  // servers.
  for (auto iter = this->dataPtr->fuelClient->Models(); iter; ++iter)
  {
    std::string fullURI = iter->Identification().UniqueName();
    std::string modelName = iter->Identification().Name();

    models[fullURI] = modelName;
  }

  return models;
}
