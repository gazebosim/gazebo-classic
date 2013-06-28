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

#include <iostream>
using namespace std;

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("wirelessTransmitter", WirelessTransmitter)

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: Sensor(sensors::OTHER)
{
  this->active = false;
  srand((unsigned)time(NULL));
}

/////////////////////////////////////////////////
WirelessTransmitter::~WirelessTransmitter()
{
}

//////////////////////////////////////////////////                              
std::string WirelessTransmitter::GetTopic() const                               
{                                                                               
  std::string topicName = "~/";                                                 
  topicName += this->parentName + "/" + this->GetName() + "/receiver";          
  boost::replace_all(topicName, "::", "/");                                     
                                                                                
  return topicName;                                                             
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->entity = this->world->GetEntity(this->parentName);
  this->pub = this->node->Advertise<msgs::Pose>(this->GetTopic(), 30);

  if (this->sdf->HasElement("transmitter"))
  {
    sdf::ElementPtr transElem = this->sdf->GetElement("transmitter");

    if (transElem->HasElement("essid"))
    {
      this->essid = transElem->GetValueString("essid");
      //cout << "ESSID: " << this->essid << endl;
    }

    if (transElem->HasElement("frequency"))
    {
      this->freq = transElem->GetValueDouble("frequency");
      //cout << "Freq: " << this->freq << endl;
    }
  }
}

/////////////////////////////////////////////////
double WirelessTransmitter::GetFreq()
{
  return this->freq;
}

/////////////////////////////////////////////////
std::string WirelessTransmitter::GetESSID()
{
  return this->essid;
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
  if (this->pub)                                                           
  {                                                                             
    msgs::Pose msg;
    msgs::Set(&msg, entity->GetWorldPose());
                                                                              
    this->pub->Publish(msg);
  }
}

double WirelessTransmitter::GetSignalStrength(const math::Pose _transmitter,
                                              const math::Pose _receiver)
{
  double distance = _transmitter.pos.Distance(_receiver.pos);
  
  if (distance > 0.0)
  {
    return min(1.0, 1.0 / distance);
  }
  else
  {
    return 0.0;
  }
}
