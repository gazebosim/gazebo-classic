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

#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/WirelessTransceiver.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("Wireless_transceiver", WirelessTransceiver)

/////////////////////////////////////////////////
WirelessTransceiver::WirelessTransceiver()
  : Sensor(sensors::OTHER)
{
  this->active = false;
  this->gain = 2.5;
  this->power = 14.5;
}

//////////////////////////////////////////////////
std::string WirelessTransceiver::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/transceiver";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

/////////////////////////////////////////////////
void WirelessTransceiver::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->transceiverElem = this->sdf->GetElement("transceiver");
  if (!this->transceiverElem)
  {
    gzerr << "Transceiver sensor is missing <transceiver> SDF element";
    return;
  }

  this->gain = transceiverElem->Get<double>("gain");
  this->power = transceiverElem->Get<double>("power");

  if (this->gain < 0)
  {
    gzerr << "Wireless transceiver gain must be > 0. Current value is ["
      << this->gain << "]\n";
    return;
  }

  if (this->power < 0)
  {
    gzerr << "Wireless transceiver power must be > 0. Current value is ["
      << this->power << "]\n";
    return;
  }
}

//////////////////////////////////////////////////
void WirelessTransceiver::Init()
{
  Sensor::Init();
}

/////////////////////////////////////////////////
void WirelessTransceiver::Fini()
{
  this->pub.reset();
  Sensor::Fini();
}

/////////////////////////////////////////////////
double WirelessTransceiver::GetPower() const
{
  return this->power;
}

/////////////////////////////////////////////////
double WirelessTransceiver::GetGain() const
{
  return this->gain;
}
