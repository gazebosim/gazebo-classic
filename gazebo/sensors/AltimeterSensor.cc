/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <boost/algorithm/string.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/AltimeterSensorPrivate.hh"
#include "gazebo/sensors/AltimeterSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("altimeter", AltimeterSensor)

/////////////////////////////////////////////////
AltimeterSensor::AltimeterSensor()
: Sensor(*new AltimeterSensorPrivate, sensors::OTHER)
{
  this->dataPtr = std::static_pointer_cast<AltimeterSensorPrivate>(this->dPtr);
}

/////////////////////////////////////////////////
AltimeterSensor::~AltimeterSensor()
{
}

/////////////////////////////////////////////////
void AltimeterSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
std::string AltimeterSensor::GetTopic() const
{
  std::string topicName = "~/" + this->ParentName() + '/' + this->Name();
  if (this->dataPtr->sdf->HasElement("topic"))
    topicName += '/' + this->dataPtr->sdf->Get<std::string>("topic");
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

/////////////////////////////////////////////////
void AltimeterSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->dataPtr->world->GetEntity(
      this->ParentName());
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->dataPtr->altPub =
    this->dataPtr->node->Advertise<msgs::Altimeter>(this->Topic(), 50);

  // Parse sdf noise parameters
  sdf::ElementPtr altElem = this->dataPtr->sdf->GetElement("altimeter");

  if (!altElem)
  {
    gzerr << "Missing <altimeter> element in sensor["
      << this->Name() << "]\n";
  }
  else
  {
    // Load altimeter vertical position noise
    if (altElem->HasElement("vertical_position") &&
        altElem->GetElement("vertical_position")->HasElement("noise"))
    {
      sdf::ElementPtr vertPosElem = altElem->GetElement("vertical_position");
      this->dPtr->noises[ALTIMETER_POSITION_NOISE_METERS] =
      NoiseFactory::NewNoiseModel(vertPosElem->GetElement("noise"));
    }

    // Load altimeter vertical velocity noise
    if (altElem->HasElement("vertical_velocity") &&
        altElem->GetElement("vertical_velocity")->HasElement("noise"))
    {
      sdf::ElementPtr vertVelElem = altElem->GetElement("vertical_velocity");
      this->dataPtr->noises[ALTIMETER_VELOCITY_NOISE_METERS_PER_S] =
        NoiseFactory::NewNoiseModel(vertVelElem->GetElement("noise"));
    }

    // Initialise reference altitude
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->altMsg.set_vertical_reference((this->dataPtr->pose +
         this->dataPtr->parentLink->GetWorldPose().Ign()).Pos().Z());
  }
}

/////////////////////////////////////////////////
void AltimeterSensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->parentLink.reset();
}

//////////////////////////////////////////////////
void AltimeterSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool AltimeterSensor::UpdateImpl(bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Get latest pose information
  if (this->dataPtr->parentLink)
  {
    ignition::math::Pose3d parentPose =
      this->dataPtr->parentLink->GetWorldPose().Ign();

    // Get pose in gazebo reference frame
    ignition::math::Pose3d altPose = this->dataPtr->pose + parentPose;

    ignition::math::Vector3d altVel =
      this->dataPtr->parentLink->GetWorldLinearVel(
          this->dataPtr->pose.Pos()).Ign();

    // Apply noise to the position and velocity
    if (this->dataPtr->noises.find(ALTIMETER_POSITION_NOISE_METERS) !=
        this->dataPtr->noises.end())
    {
      this->dataPtr->altMsg.set_vertical_position(
          this->dataPtr->noises[ALTIMETER_POSITION_NOISE_METERS]->Apply(
            altPose.Pos().Z() - this->dataPtr->altMsg.vertical_reference()));
    }
    else
    {
      this->dataPtr->altMsg.set_vertical_position(
          altPose.Pos().Z() - this->dataPtr->altMsg.vertical_reference());
    }

    if (this->dataPtr->noises.find(ALTIMETER_VELOCITY_NOISE_METERS_PER_S) !=
        this->dataPtr->noises.end())
    {
      this->dataPtr->altMsg.set_vertical_velocity(
          this->dataPtr->noises[ALTIMETER_VELOCITY_NOISE_METERS_PER_S]->Apply(
            altVel.Z()));
    }
    else
    {
      this->dataPtr->altMsg.set_vertical_velocity(altVel.Z());
    }
  }

  // Save the time of the measurement
  msgs::Set(this->dataPtr->altMsg.mutable_time(),
      this->dataPtr->world->GetSimTime());

  // Publish the message if needed
  if (this->dataPtr->altPub)
    this->dataPtr->altPub->Publish(this->dataPtr->altMsg);

  return true;
}

//////////////////////////////////////////////////
double AltimeterSensor::Altitude() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->altMsg.vertical_position();
}

//////////////////////////////////////////////////
double AltimeterSensor::VerticalVelocity() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->altMsg.vertical_velocity();
}

//////////////////////////////////////////////////
double AltimeterSensor::ReferenceAltitude() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->altMsg.vertical_reference();
}

//////////////////////////////////////////////////
void AltimeterSensor::SetReferenceAltitude(const double _refAlt)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Recompute the last altitude with the new vertical reference height
  this->dataPtr->altMsg.set_vertical_position(
      this->dataPtr->altMsg.vertical_position() -
      (_refAlt - this->dataPtr->altMsg.vertical_reference()));

  // Save the new reference height
  this->dataPtr->altMsg.set_vertical_reference(_refAlt);
}
