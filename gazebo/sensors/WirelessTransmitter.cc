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
/* Desc: Wireless transmitter 
 * Author: Carlos Agüero 
 * Date: 24 June 2013
 */

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("wirelessTransmitter", WirelessTransmitter)

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: Sensor(sensors::OTHER)
{
  this->active = false;
}

/////////////////////////////////////////////////
WirelessTransmitter::~WirelessTransmitter()
{
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName, sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->entity = this->world->GetEntity(this->parentName);
}

/////////////////////////////////////////////////
void WirelessTransmitter::Fini()
{
  Sensor::Fini();
  this->entity.reset();
}

//////////////////////////////////////////////////
void WirelessTransmitter::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void WirelessTransmitter::UpdateImpl(bool /*_force*/)
{

}
