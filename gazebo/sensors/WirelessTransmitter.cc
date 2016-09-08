/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <ignition/math/Rand.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/sensors/WirelessTransmitterPrivate.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"

using namespace gazebo;
using namespace sensors;
using namespace physics;

GZ_REGISTER_STATIC_SENSOR("wireless_transmitter", WirelessTransmitter)

const double WirelessTransmitterPrivate::NEmpty = 6;
const double WirelessTransmitterPrivate::NObstacle = 12.0;
const double WirelessTransmitterPrivate::ModelStdDev = 6.0;
const double WirelessTransmitterPrivate::Step = 1.0;
const double WirelessTransmitterPrivate::MaxRadius = 10.0;

/////////////////////////////////////////////////
WirelessTransmitter::WirelessTransmitter()
: WirelessTransceiver(),
  dataPtr(new WirelessTransmitterPrivate)
{
}

/////////////////////////////////////////////////
WirelessTransmitter::~WirelessTransmitter()
{
}

/////////////////////////////////////////////////
void WirelessTransmitter::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);

  sdf::ElementPtr transceiverElem =
    this->sdf->GetElement("transceiver");

  this->dataPtr->visualize = this->sdf->Get<bool>("visualize");
  this->dataPtr->essid = transceiverElem->Get<std::string>("essid");
  this->dataPtr->freq = transceiverElem->Get<double>("frequency");

  if (this->dataPtr->freq < 0)
  {
    gzerr << "Attempting to set negative transmitter frequency of["
      << this->dataPtr->freq << "]. Using 1.";
    this->dataPtr->freq = 1.0;
  }

  this->pub =
    this->node->Advertise<msgs::PropagationGrid>(this->Topic(), 30);
  GZ_ASSERT(this->pub != nullptr,
      "wirelessTransmitterSensor did not get a valid publisher pointer");
}

//////////////////////////////////////////////////
void WirelessTransmitter::Init()
{
  WirelessTransceiver::Init();

  // This ray will be used in SignalStrength() for checking obstacles
  // between the transmitter and a given point.
  this->dataPtr->testRay = boost::dynamic_pointer_cast<RayShape>(
      this->world->GetPhysicsEngine()->CreateShape("ray",
        CollisionPtr()));
}

//////////////////////////////////////////////////
bool WirelessTransmitter::UpdateImpl(const bool /*_force*/)
{
  this->referencePose = this->pose +
    this->parentEntity.lock()->GetWorldPose().Ign();

  if (this->dataPtr->visualize)
  {
    msgs::PropagationGrid msg;
    ignition::math::Pose3d pos;
    ignition::math::Pose3d worldPose;
    double strength;
    msgs::PropagationParticle *p;

    // Iterate using a rectangular grid, but only choose the points within
    // a circunference of radius MaxRadius
    for (double x = -this->dataPtr->MaxRadius;
         x <= this->dataPtr->MaxRadius; x += this->dataPtr->Step)
    {
      for (double y = -this->dataPtr->MaxRadius;
           y <= this->dataPtr->MaxRadius; y += this->dataPtr->Step)
      {
        pos.Set(x, y, 0.0, 0, 0, 0);

        worldPose = pos + this->referencePose;

        if (this->referencePose.Pos().Distance(worldPose.Pos()) <=
            this->dataPtr->MaxRadius)
        {
          // For the propagation model assume the receiver antenna has the same
          // gain as the transmitter
          strength = this->SignalStrength(worldPose, this->Gain());

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
std::string WirelessTransmitter::ESSID() const
{
  return this->dataPtr->essid;
}

/////////////////////////////////////////////////
double WirelessTransmitter::Freq() const
{
  return this->dataPtr->freq;
}

/////////////////////////////////////////////////
double WirelessTransmitter::SignalStrength(
    const ignition::math::Pose3d &_receiver,
    const double _rxGain)
{
  std::string entityName;
  double dist;
  ignition::math::Vector3d end = _receiver.Pos();
  ignition::math::Vector3d start = this->referencePose.Pos();

  // Avoid computing the intersection of coincident points
  // This prevents an assertion in bullet (issue #849)
  if (start == end)
  {
    end.Z() += 0.00001;
  }

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(
        this->world->GetPhysicsEngine()->GetPhysicsUpdateMutex()));

  // Compute the value of n depending on the obstacles between Tx and Rx
  double n = WirelessTransmitterPrivate::NEmpty;

  // Looking for obstacles between start and end points
  this->dataPtr->testRay->SetPoints(start, end);
  this->dataPtr->testRay->GetIntersection(dist, entityName);

  // ToDo: The ray intersects with my own collision model. Fix it.
  if (entityName != "")
  {
    n = WirelessTransmitterPrivate::NObstacle;
  }

  double distance = std::max(1.0,
      this->referencePose.Pos().Distance(_receiver.Pos()));
  double x = std::abs(ignition::math::Rand::DblNormal(0.0,
        WirelessTransmitterPrivate::ModelStdDev));
  double wavelength = common::SpeedOfLight / (this->Freq() * 1000000);

  // Hata-Okumara propagation model
  double rxPower = this->Power() + this->Gain() + _rxGain - x +
      20 * log10(wavelength) - 20 * log10(4 * M_PI) - 10 * n * log10(distance);

  return rxPower;
}

/////////////////////////////////////////////////
double WirelessTransmitter::ModelStdDev() const
{
  return WirelessTransmitterPrivate::ModelStdDev;
}
