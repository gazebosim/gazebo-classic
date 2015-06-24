/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/AltimeterSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("altimeter", AltimeterSensor)

/////////////////////////////////////////////////
AltimeterSensor::AltimeterSensor()
: Sensor(sensors::OTHER)
{
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
void AltimeterSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->altPub = this->node->Advertise<msgs::Altimeter>(this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr altElem = this->sdf->GetElement("altimeter");

  // Load altimeter vertical position noise
  {
    sdf::ElementPtr vertPosElem = altElem->GetElement("vertical_position");
    this->noises[ALTIMETER_POSITION_NOISE_METERS] = NoiseFactory::NewNoiseModel(
      vertPosElem->GetElement("noise"));
  }

  // Load altimeter vertical velocity noise
  {
    sdf::ElementPtr vertVelElem = altElem->GetElement("vertical_velocity");
    this->noises[ALTIMETER_VELOCITY_NOISE_METERS_PER_S] = 
      NoiseFactory::NewNoiseModel(vertVelElem->GetElement("noise"));
  }

  // Initialise reference altitude
  boost::mutex::scoped_lock lock(this->mutex);
  this->altMsg.set_vertical_reference(
     (this->pose + this->parentLink->GetWorldPose()).pos.z);
}

/////////////////////////////////////////////////
void AltimeterSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void AltimeterSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool AltimeterSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Get latest pose information
  if (this->parentLink)
  {
    // Get pose in gazebo reference frame
    math::Pose altPose = this->pose + this->parentLink->GetWorldPose();
    math::Vector3 altVel = this->parentLink->GetWorldLinearVel(this->pose.pos);

    // Apply noise to the position and velocity 
    this->altMsg.set_vertical_position(
      this->noises[ALTIMETER_POSITION_NOISE_METERS]->Apply(
        altPose.pos.z - this->altMsg.vertical_reference()));
    this->altMsg.set_vertical_velocity( 
      this->noises[ALTIMETER_VELOCITY_NOISE_METERS_PER_S]->Apply(altVel.z));
  }

  // Save the time of the measurement
  msgs::Set(this->altMsg.mutable_time(), this->world->GetSimTime());

  // Publish the message if needed
  if (this->altPub)
    this->altPub->Publish(this->altMsg);

  return true;
}

//////////////////////////////////////////////////
double AltimeterSensor::GetVerticalPosition()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->altMsg.vertical_position(); 
}

//////////////////////////////////////////////////
double AltimeterSensor::GetVerticalVelocity()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->altMsg.vertical_velocity();
}

//////////////////////////////////////////////////
double AltimeterSensor::GetReferenceAltitude()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->altMsg.vertical_reference();
}

//////////////////////////////////////////////////
void AltimeterSensor::SetReferenceAltitude(double _refAlt)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Recompute the last altitude with the new vertical reference height
  this->altMsg.set_vertical_position(this->altMsg.vertical_position() 
    - (_refAlt - this->altMsg.vertical_reference()));

  // Save the new reference height
  this->altMsg.set_vertical_reference(_refAlt);
}