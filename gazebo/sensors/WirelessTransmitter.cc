/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
const double WirelessTransmitter::Step = 1.0;
const double WirelessTransmitter::MaxRadius = 10.0;

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: WirelessTransceiver()
{
  this->active = false;
  this->visualize = false;
  this->essid = "MyESSID";
  this->freq = 2442.0;
}

/////////////////////////////////////////////////
WirelessTransmitter::~WirelessTransmitter()
{
  this->testRay.reset();
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);

  sdf::ElementPtr transceiverElem = this->sdf->GetElement("transceiver");

  this->visualize = this->sdf->Get<bool>("visualize");
  this->essid = transceiverElem->Get<std::string>("essid");
  this->freq = transceiverElem->Get<double>("frequency");

  if (this->freq < 0)
  {
    gzthrow("Wireless transmitter frequency must be > 0. Current value is ["
      << this->freq << "]");
    return;
  }

  this->pub = this->node->Advertise<msgs::PropagationGrid>(this->GetTopic(),
        30);
  GZ_ASSERT(this->pub != NULL,
      "wirelessTransmitterSensor did not get a valid publisher pointer");
}

//////////////////////////////////////////////////
void WirelessTransmitter::Init()
{
  WirelessTransceiver::Init();

  // This ray will be used in GetSignalStrength() for checking obstacles
  // between the transmitter and a given point.
  this->testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));
}

//////////////////////////////////////////////////
bool WirelessTransmitter::UpdateImpl(bool /*_force*/)
{
  this->referencePose =
      this->pose + this->parentEntity.lock()->GetWorldPose();

  if (this->visualize)
  {
    msgs::PropagationGrid msg;
    math::Pose pos;
    math::Pose worldPose;
    double strength;
    msgs::PropagationParticle *p;

    // Iterate using a rectangular grid, but only choose the points within
    // a circunference of radius MaxRadius
    for (double x = -this->MaxRadius; x <= this->MaxRadius; x += this->Step)
    {
      for (double y = -this->MaxRadius; y <= this->MaxRadius; y += this->Step)
      {
        pos.Set(x, y, 0.0, 0, 0, 0);

        worldPose = pos + this->referencePose;

        if (this->referencePose.pos.Distance(worldPose.pos) <= this->MaxRadius)
        {
          // For the propagation model assume the receiver antenna has the same
          // gain as the transmitter
          strength = this->GetSignalStrength(worldPose, this->GetGain());

          // Add a new particle to the grid
          p = msg.add_particle();
          p->set_x(x);
          p->set_y(y);
          p->set_signal_level(strength);
        }
      }
    }
    this->pub->Publish(msg);
  }

  return true;
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
double WirelessTransmitter::GetSignalStrength(const math::Pose &_receiver,
    const double rxGain)
{
  std::string entityName;
  double dist;
  math::Vector3 end = _receiver.pos;
  math::Vector3 start = this->referencePose.pos;

  // Avoid computing the intersection of coincident points
  // This prevents an assertion in bullet (issue #849)
  if (start == end)
  {
    end.z += 0.00001;
  }

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(world->GetPhysicsEngine()->
      GetPhysicsUpdateMutex()));

  // Compute the value of n depending on the obstacles between Tx and Rx
  double n = NEmpty;

  // Looking for obstacles between start and end points
  this->testRay->SetPoints(start, end);
  this->testRay->GetIntersection(dist, entityName);

  // ToDo: The ray intersects with my own collision model. Fix it.
  if (entityName != "")
  {
    n = NObstacle;
  }

  double distance = std::max(1.0,
      this->referencePose.pos.Distance(_receiver.pos));
  double x = abs(math::Rand::GetDblNormal(0.0, ModelStdDesv));
  double wavelength = common::SpeedOfLight / (this->GetFreq() * 1000000);

  // Hata-Okumara propagation model
  double rxPower = this->GetPower() + this->GetGain() + rxGain - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}
