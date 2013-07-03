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

const double WirelessTransmitter::N_EMPTY = 10.0;
const double WirelessTransmitter::N_OBSTACLE = 20.0;
const double WirelessTransmitter::MODEL_STD_DESV = 3.0;
const double WirelessTransmitter::XLIMIT = 10.0;
const double WirelessTransmitter::YLIMIT = 10.0;
const double WirelessTransmitter::STEP = 1.0;

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: WirelessTransceiver()
{
  this->active = false;
  srand((unsigned)time(NULL));
}

/////////////////////////////////////////////////
WirelessTransmitter::~WirelessTransmitter()
{
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
  WirelessTransceiver::Fini();
  this->entity.reset();
}

/////////////////////////////////////////////////
std::string WirelessTransmitter::GetESSID()
{
  return this->essid;
}

/////////////////////////////////////////////////
math::Pose WirelessTransmitter::GetPose() const
{
  return entity->GetWorldPose();
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
  double n = N_EMPTY;

  /// \brief Ray used to test for collisions when placing entities.
  physics::RayShapePtr testRay;
  
  testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);
  testRay.reset();

  //ToDo: The ray intersects with my own collision model. Fix it.
  if (dist > 0 && entityName != "")
  {
    n = N_OBSTACLE;
  }

  double distance = txPos.pos.Distance(_receiver.pos);  
  double x = math::Rand::GetDblNormal(0.0, MODEL_STD_DESV);
  double wavelength = C / this->GetFreq();

  // Hata-Okumara propagation model
  double rxPower = this->GetPower() + this->GetGain() + gainRx - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}
