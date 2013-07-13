/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/sensors/GpsSensor.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/physics/World.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gps", GpsSensor)

/////////////////////////////////////////////////
GpsSensor::GpsSensor()
: Sensor(sensors::OTHER)
{
}

/////////////////////////////////////////////////
GpsSensor::~GpsSensor()
{
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName, sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void GpsSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = this->world->GetEntity(this->parentName);

  // Todo: parse sdf noise parameters
}

/////////////////////////////////////////////////
void GpsSensor::Fini()
{
  Sensor::Fini();
  this->parentEntity.reset();
  this->sphericalCoordinates.reset();
}

//////////////////////////////////////////////////
void GpsSensor::Init()
{
  Sensor::Init();
  
  this->sphericalCoordinates = this->world->GetSphericalCoordinates();
}

//////////////////////////////////////////////////
void GpsSensor::UpdateImpl(bool /*_force*/)
{
}
