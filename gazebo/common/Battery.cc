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

  this->SetUpdateFunc(boost::bind(&Battery::UpdateDefault, this, _1, _2));
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
  this->dataPtr->powerLoads.clear();
}

/////////////////////////////////////////////////
uint32_t Battery::AddConsumer()
{
  uint32_t consumerId = this->dataPtr->powerLoads.size();
  this->dataPtr->powerLoads[consumerId] = 0.0;
  return consumerId;
}

/////////////////////////////////////////////////
void Battery::RemoveConsumer(uint32_t _consumerId)
{
  this->dataPtr->powerLoads.erase(_consumerId);
}

/////////////////////////////////////////////////
bool Battery::SetPowerLoad(const uint32_t _consumerId, const double _powerLoad)
{
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
const std::map<uint32_t, double>& Battery::PowerLoads() const
{
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
      this->dataPtr->updateFunc(this->dataPtr->realVoltage,
                                this->dataPtr->powerLoads);
}

/////////////////////////////////////////////////
double Battery::UpdateDefault(const double _voltage,
    const std::map<uint32_t, double> &/*_powerLoads*/)
{
  // Ideal battery
  return _voltage;
}

/////////////////////////////////////////////////
void Battery::SetUpdateFunc(
    std::function<double (double, const std::map<uint32_t, double> &)>
    _updateFunc)
{
  this->dataPtr->updateFunc = _updateFunc;
}
