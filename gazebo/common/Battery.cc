/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Console.hh"
#include "gazebo/common/BatteryPrivate.hh"
#include "gazebo/common/Battery.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
Battery::Battery()
  : dataPtr(new BatteryPrivate)
{
  this->dataPtr->realVoltage = 0.0;
  this->dataPtr->initVoltage = 0.0;
  this->dataPtr->powerLoadCounter = 0;

  this->SetUpdateFunc(std::bind(&Battery::UpdateDefault, this,
        std::placeholders::_1));
}

/////////////////////////////////////////////////
Battery::~Battery()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void Battery::Load(const sdf::ElementPtr _sdf)
{
  this->dataPtr->name = _sdf->Get<std::string>("name");

  this->UpdateParameters(_sdf);
}

/////////////////////////////////////////////////
void Battery::Init()
{
  this->dataPtr->realVoltage = this->dataPtr->initVoltage;
  this->InitConsumers();
}

//////////////////////////////////////////////////
void Battery::UpdateParameters(sdf::ElementPtr _sdf)
{
  this->dataPtr->initVoltage = _sdf->Get<double>("voltage");
}

//////////////////////////////////////////////////
std::string Battery::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Battery::InitConsumers()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  this->dataPtr->powerLoads.clear();
}

/////////////////////////////////////////////////
uint32_t Battery::AddConsumer()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  uint32_t newId = this->dataPtr->powerLoadCounter++;
  this->dataPtr->powerLoads[newId] = 0.0;
  return newId;
}

/////////////////////////////////////////////////
bool Battery::RemoveConsumer(uint32_t _consumerId)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  if (this->dataPtr->powerLoads.erase(_consumerId))
  {
    return true;
  }
  else
  {
    gzerr << "Invalid battery consumer id[" << _consumerId << "]\n";
    return false;
  }
}

/////////////////////////////////////////////////
bool Battery::SetPowerLoad(const uint32_t _consumerId, const double _powerLoad)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  auto iter = this->dataPtr->powerLoads.find(_consumerId);
  if (iter == this->dataPtr->powerLoads.end())
  {
    gzerr << "Invalid param value[_consumerId] : " << _consumerId << "\n";
    return false;
  }

  iter->second = _powerLoad;
  return true;
}

/////////////////////////////////////////////////
bool Battery::PowerLoad(const uint32_t _consumerId, double &_powerLoad) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  auto iter = this->dataPtr->powerLoads.find(_consumerId);
  if (iter == this->dataPtr->powerLoads.end())
  {
    gzerr << "Invalid param value[_consumerId] : " << _consumerId << "\n";
    return false;
  }

  _powerLoad = iter->second;
  return true;
}

/////////////////////////////////////////////////
const Battery::PowerLoad_M &Battery::PowerLoads() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->powerLoadsMutex);
  return this->dataPtr->powerLoads;
}

/////////////////////////////////////////////////
double Battery::Voltage() const
{
  return this->dataPtr->realVoltage;
}

/////////////////////////////////////////////////
void Battery::Update()
{
  this->dataPtr->realVoltage =
      this->dataPtr->updateFunc(shared_from_this());
}

/////////////////////////////////////////////////
double Battery::UpdateDefault(const BatteryPtr &_battery)
{
  // Ideal battery
  return _battery->Voltage();
}

/////////////////////////////////////////////////
void Battery::SetUpdateFunc(
    std::function<double (const BatteryPtr &)> _updateFunc)
{
  this->dataPtr->updateFunc = _updateFunc;
}
