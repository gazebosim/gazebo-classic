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
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace sensors;
using namespace physics;

GZ_REGISTER_STATIC_SENSOR("wireless_transmitter", WirelessTransmitter)

const double WirelessTransmitter::NEmpty = 6;
const double WirelessTransmitter::NObstacle = 12.0;
const double WirelessTransmitter::ModelStdDesv = 6.0;
const double WirelessTransmitter::XLimit = 10.0;
const double WirelessTransmitter::YLimit = 10.0;
const double WirelessTransmitter::Step = 0.25;

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: WirelessTransceiver()
{
  this->active = false;
  srand((unsigned)time(NULL));

  this->essid = "MyESSID";
  this->freq = 2442.0;
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);

  this->entity = this->world->GetEntity(this->parentName);
  GZ_ASSERT(this->entity != NULL, "Unable to get the parent entity.");

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

    for (double x = -this->XLimit; x <= this->XLimit; x += this->Step)
    {
      for (double y = -YLimit; y <= YLimit; y += Step)
      {
        pos.Set(x, y, 0.0, 0.0, 0.0, 0.0);
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
std::string WirelessTransmitter::GetESSID() const
{
  return this->essid;
}

/////////////////////////////////////////////////
double WirelessTransmitter::GetFreq() const
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
  double n = NEmpty;

  // Check for collisions between transmitter and receiver
  this->testRay = boost::static_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));
  this->testRay->SetPoints(start, end);
  this->testRay->GetIntersection(dist, entityName);
  this->testRay.reset();

  // ToDo: The ray intersects with my own collision model. Fix it.
  if (dist > 0 && entityName != "ground_plane::link::collision" &&
      entityName != "" && entityName != "wirelessReceiver::link::collision-box")
  {
    n = NObstacle;
  }

  double distance = std::max(1.0, txPos.pos.Distance(_receiver.pos));
  double x = abs(math::Rand::GetDblNormal(0.0, ModelStdDesv));
  double wavelength = common::SpeedOfLight / (this->GetFreq() * 1000000);

  // Hata-Okumara propagation model
  double rxPower = this->GetPower() + this->GetGain() + rxGain - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}
