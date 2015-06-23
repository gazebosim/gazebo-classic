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
#include "gazebo/sensors/noise/Noise.hh"
#include "gazebo/sensors/MagnetometerSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("magnetometer", MagnetometerSensor)

/////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor()
: Sensor(sensors::OTHER)
{
}

/////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor()
{
}

/////////////////////////////////////////////////
void MagnetometerSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void MagnetometerSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->lastMagnetometerMsg.set_link_name(this->parentName);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->magPub = this->node->Advertise<msgs::Magnetometer>(this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr magElem = this->sdf->GetElement("magnetometer");

  // Load magnetic field noise parameters
  {
    sdf::ElementPtr fieldElem = magElem->GetElement("field");
    this->noises[MAGNETOMETER_X_NOISE_TESLA] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("x")->GetElement("noise"));
    this->noises[MAGNETOMETER_Y_NOISE_TESLA] =  NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("y")->GetElement("noise"));
    this->noises[MAGNETOMETER_Z_NOISE_TESLA] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("z")->GetElement("noise"));
  }
}

/////////////////////////////////////////////////
void MagnetometerSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void MagnetometerSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool MagnetometerSensor::UpdateImpl(bool /*_force*/)
{
  // Get latest pose information
  if (this->parentLink)
  {
    // Get pose in gazebo reference frame
    math::Pose magPose = this->pose + this->parentLink->GetWorldPose();

    // Get the reference magnetic field
    math::Vector3 M = this->world->GetPhysicsEngine()->GetMagneticField();

    // Rotate the magnetic field into the body frame
    M = magPose.rot.GetInverse().RotateVector(M);

    // Apply position noise before converting to global frame
    M.x = this->noises[MAGNETOMETER_X_NOISE_TESLA]->Apply(M.x);
    M.y = this->noises[MAGNETOMETER_Y_NOISE_TESLA]->Apply(M.y);
    M.z = this->noises[MAGNETOMETER_Z_NOISE_TESLA]->Apply(M.z);

    // Set the IMU orientation
    msgs::Set(this->lastMagMsg.mutable_field_tesla(),M);
  }

  // Save the time of the measurement
  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->lastMagMsg.mutable_time(), this->lastMeasurementTime);

  // Publish the message if needed
  if (this->magPub)
    this->magPub->Publish(this->lastMagMsg);

  return true;
}

//////////////////////////////////////////////////
math::Vector3 MagnetometerSensor::GetMagneticField() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->magMsg.field_tesla());
}