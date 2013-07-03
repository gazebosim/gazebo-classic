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

#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace sensors;
using namespace physics;

GZ_REGISTER_STATIC_SENSOR("wirelessTransmitter", WirelessTransmitter)

const double WirelessTransmitter::XLIMIT = 10.0;
const double WirelessTransmitter::YLIMIT = 10.0;
const double WirelessTransmitter::STEP = 1.0;
const unsigned int WirelessTransmitter::C = 300000000;

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
  this->pub = this->node->Advertise<msgs::PropagationGrid>(
      this->GetTopic(), 30);

  if (this->sdf->HasElement("transceiver"))
  {
    sdf::ElementPtr transElem = this->sdf->GetElement("transceiver");

    if (transElem->HasElement("essid"))
    {
      this->essid = transElem->GetValueString("essid");
    }

    if (transElem->HasElement("frequency"))
    {
      this->freq = transElem->GetValueDouble("frequency");
    }

    if (transElem->HasElement("power"))
    {
      this->power = transElem->GetValueDouble("power");
    }

    if (transElem->HasElement("gain"))
    {
      this->gain = transElem->GetValueDouble("gain");
    }
  }
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
    math::Pose pos;
    double strength;
    msgs::PropagationParticle *p;
    
    double x = -XLIMIT;
    double y = -YLIMIT;
    while (x <= XLIMIT)
    {
      while (y <= YLIMIT)
      {
        pos = math::Pose(x, y, 0, 0, 0, 0);
        strength = this->GetSignalStrength(pos);

        // Add a new particle to the grid
        p = msg.add_particle();
        p->set_x(x);
        p->set_y(y);
        p->set_signal_level(strength);
        
        y += STEP;
      }
      x += STEP ;
      y = -YLIMIT;
    }                                                                          
    this->pub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void WirelessTransmitter::Fini()
{
  Sensor::Fini();
  this->entity.reset();
}

/////////////////////////////////////////////////
std::string WirelessTransmitter::GetESSID()
{
  return this->essid;
}

/////////////////////////////////////////////////
double WirelessTransmitter::GetFreq()
{
  return this->freq;
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
double WirelessTransmitter::GetSignalStrength(const math::Pose _receiver)
{
  math::Pose txPos = this->GetPose();
  std::string entityName;
  double dist;
  math::Vector3 start = txPos.pos;
  math::Vector3 end = _receiver.pos;

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(world->GetPhysicsEngine()->
      GetPhysicsUpdateMutex()));

  // We assume that rxGain is equals to txGain
  double gainRx = this->gain;
  
  // Compute the value of n depending on the obstacles between Tx and Rx
  double n = 10;

  /// \brief Ray used to test for collisions when placing entities.
  physics::RayShapePtr testRay;
  
  testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);
  testRay.reset();

  //ToDo: The ray will intersect with my own collision model. Fix it.
  if (dist > 0 && entityName != "")
  {
    n = 20;
  }

  double distance = txPos.pos.Distance(_receiver.pos);  
  double x = math::Rand::GetDblNormal(0.0, 3.0);
  double wavelength = C / this->GetFreq();

  // Propagation model
  double rxPower = this->GetPower() + this->GetGain() + gainRx - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}
