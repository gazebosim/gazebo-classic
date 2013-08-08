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

GZ_REGISTER_STATIC_SENSOR("wireless_transmitter", WirelessTransmitter)

const double WirelessTransmitter::N_EMPTY = 6;
const double WirelessTransmitter::N_OBSTACLE = 12.0;
const double WirelessTransmitter::MODEL_STD_DESV = 6.0;
const double WirelessTransmitter::XLIMIT = 10.0;
const double WirelessTransmitter::YLIMIT = 10.0;
const double WirelessTransmitter::STEP = 0.25;

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: WirelessTransceiver()
{
  this->active = false;
  srand((unsigned)time(NULL));
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);

  this->entity = this->world->GetEntity(this->parentName);

  if (this->sdf->HasElement("transceiver"))
  {
    sdf::ElementPtr transElem = this->sdf->GetElement("transceiver");

    if (transElem->HasElement("essid"))
    {
      this->essid = transElem->Get<std::string>("essid");
    }

    if (transElem->HasElement("frequency"))
    {
      this->freq = transElem->Get<double>("frequency");
    }
  }

  this->pub = this->node->Advertise<msgs::PropagationGrid>(this->GetTopic(),
        30);
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

    for (double x = -XLIMIT; x <= XLIMIT; x += STEP)
    {
      for (double y = -YLIMIT; y <= YLIMIT; y += STEP)
      {
        pos = math::Pose(x, y, 0, 0, 0, 0);
        // For the propagation model assume the receiver antenna has the same
        // gain as the transmitter
        strength = this->GetSignalStrength(pos, this->GetGain());

        // Add a new particle to the grid
        p = msg.add_particle();
        p->set_x(x);
        p->set_y(y);
        p->set_signal_level(strength);
      }
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
double WirelessTransmitter::GetFreq()
{
  return this->freq;
}

/////////////////////////////////////////////////
math::Pose WirelessTransmitter::GetPose() const
{
  return entity->GetWorldPose();
}

/////////////////////////////////////////////////
double WirelessTransmitter::GetSignalStrength(const math::Pose &_receiver,
    const double rxGain)
{
  math::Pose txPos = this->GetPose();
  std::string entityName;
  double dist;
  math::Vector3 start = txPos.pos;
  math::Vector3 end = _receiver.pos;

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(world->GetPhysicsEngine()->
      GetPhysicsUpdateMutex()));

  // Compute the value of n depending on the obstacles between Tx and Rx
  double n = N_EMPTY;

  // Ray used to test for collisions when placing entities
  physics::RayShapePtr testRay;

  testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);
  testRay.reset();

  // ToDo: The ray intersects with my own collision model. Fix it.
  if (dist > 0 && entityName != "ground_plane::link::collision" &&
      entityName != "" && entityName != "wirelessReceiver::link::collision-box")
  {
    n = N_OBSTACLE;
  }

  double distance = std::max(1.0, txPos.pos.Distance(_receiver.pos));
  double x = abs(math::Rand::GetDblNormal(0.0, MODEL_STD_DESV));
  double wavelength = C / (this->GetFreq() * 1000000);

  // Hata-Okumara propagation model
  double rxPower = this->GetPower() + this->GetGain() + rxGain - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}
