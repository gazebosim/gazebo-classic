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

#include "gazebo/physics/physics.hh"

#include <iostream>
using namespace std;

using namespace gazebo;
using namespace sensors;
using namespace physics;

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
  topicName += this->parentName + "/" + this->GetName() + "/transmitter";          
  boost::replace_all(topicName, "::", "/");                                     
                                                                                
  return topicName;                                                             
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->entity = this->world->GetEntity(this->parentName);
  this->pub = this->node->Advertise<msgs::PropagationGrid>(this->GetTopic(), 30);

  if (this->sdf->HasElement("transceiver"))
  {
    sdf::ElementPtr transElem = this->sdf->GetElement("transceiver");

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

    if (transElem->HasElement("power"))
    {
      this->power = transElem->GetValueDouble("power");
      //cout << "Power: " << this->gain << endl;
    }

    if (transElem->HasElement("gain"))
    {
      this->gain = transElem->GetValueDouble("gain");
      //cout << "Gain: " << this->gain << endl;
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
double WirelessTransmitter::GetGain()
{
  return this->gain;
}

/////////////////////////////////////////////////
double WirelessTransmitter::GetPower()
{
  return this->power;
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
    msgs::PropagationGrid msg;
    
    double x = -10.0;
    double y = -10.0;
    for (int i = 0; i < 21; i++)
    {
      for (int j = 0; j < 21; j++)
      {
        math::Pose pos = math::Pose(x, y, 0, 0, 0, 0);

        double strength = WirelessTransmitter::GetSignalStrength(
          this->GetPose(), pos);

        msgs::PropagationParticle *p = msg.add_particle();
        p->set_x(x);
        p->set_y(y);
        p->set_signal_level(strength);
        
        x += 1.0;
      }
      x = -10.0;
      y += 1.0;
    }                                                                          
    this->pub->Publish(msg);
  }
}

double WirelessTransmitter::GetSignalStrength(const math::Pose _transmitter,
        const math::Pose _receiver)
{
  double _powTx = 14.5;
  double _gainTx = 2.5;
  double _gainRx = 2.5;
  double _freq = 2442.0;
  
  // Compute the value of N deppending on the number of objects between
  // transmitter and receiver
  double N = 10;

  /// \brief Ray used to test for collisions when placing entities.
  /*physics::RayShapePtr testRay;
  physics::WorldPtr world = physics::get_world("default");
  if (world == NULL)
  {
    cout << "World is NULL\n" ;
    return 0.0;
  }

  testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  std::string entityName;
  double dist;
  math::Vector3 start = _transmitter.pos;
  math::Vector3 end = _receiver.pos;
  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);
  cout << dist << "," << entityName << endl;

  testRay.reset();*/

  double distance = _transmitter.pos.Distance(_receiver.pos);
  /*if (dist < distance)
  {
    N = 20;
  }*/
  
  double x = math::Rand::GetDblNormal(0.0, 3.0);
  double wavelength = 300000000 / _freq;

  double rxPower = _powTx + _gainTx + _gainRx - x + 20 * log10(wavelength) -
            20 * log10(4 * M_PI) - 10 * N * log10(distance);

  return rxPower;
}
