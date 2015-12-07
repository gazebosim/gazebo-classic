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
#include <ignition/math/Pose3.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/MagnetometerSensorPrivate.hh"
#include "gazebo/sensors/MagnetometerSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("magnetometer", MagnetometerSensor)

/////////////////////////////////////////////////
MagnetometerSensor::MagnetometerSensor()
: Sensor(*new MagnetometerSensorPrivate, sensors::OTHER),
  dataPtr(std::static_pointer_cast<MagnetometerSensorPrivate>(this->sensorDPtr))
{
}

/////////////////////////////////////////////////
MagnetometerSensor::~MagnetometerSensor()
{
}

/////////////////////////////////////////////////
void MagnetometerSensor::Load(const std::string &_worldName,
    sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
std::string MagnetometerSensor::GetTopic() const
{
  std::string topicName = "~/" + this->ParentName() + '/' + this->Name();
  if (this->dataPtr->sdf->HasElement("topic"))
    topicName += '/' + this->dataPtr->sdf->Get<std::string>("topic");
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

/////////////////////////////////////////////////
void MagnetometerSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity =
    this->dataPtr->world->GetEntity(this->ParentName());
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->dataPtr->magPub = this->dataPtr->node->Advertise<msgs::Magnetometer>(
      this->GetTopic(), 50);

  // Parse sdf noise parameters
  sdf::ElementPtr magElem = this->dataPtr->sdf->GetElement("magnetometer");

  if (!magElem)
  {
    gzerr << "Missing <magnetometer> element in sensor["
      << this->Name() << "]\n";
  }
  else
  // Load magnetic field noise parameters
  {
    if (magElem->HasElement("x") &&
        magElem->GetElement("x")->HasElement("noise"))
    {
      this->dataPtr->noises[MAGNETOMETER_X_NOISE_TESLA] =
        NoiseFactory::NewNoiseModel(
            magElem->GetElement("x")->GetElement("noise"));
    }

    if (magElem->HasElement("y") &&
        magElem->GetElement("y")->HasElement("noise"))
    {
      this->dataPtr->noises[MAGNETOMETER_Y_NOISE_TESLA] =
        NoiseFactory::NewNoiseModel(
            magElem->GetElement("y")->GetElement("noise"));
    }

    if (magElem->HasElement("z") &&
        magElem->GetElement("z")->HasElement("noise"))
    {
      this->dataPtr->noises[MAGNETOMETER_Z_NOISE_TESLA] =
        NoiseFactory::NewNoiseModel(
            magElem->GetElement("z")->GetElement("noise"));
    }
  }
}

/////////////////////////////////////////////////
void MagnetometerSensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->parentLink.reset();
}

//////////////////////////////////////////////////
void MagnetometerSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool MagnetometerSensor::UpdateImpl(bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Get latest pose information
  if (this->dataPtr->parentLink)
  {
    // Get pose in gazebo reference frame
    ignition::math::Pose3d magPose =
      this->dataPtr->pose + this->dataPtr->parentLink->GetWorldPose().Ign();

    // Get the reference magnetic field
    ignition::math::Vector3d field =
      this->dataPtr->world->GetPhysicsEngine()->MagneticField();

    // Rotate the magnetic field into the body frame
    field = magPose.Rot().Inverse().RotateVector(field);

    // Apply magnetometer noise after converting to body frame
    if (this->dataPtr->noises.find(MAGNETOMETER_X_NOISE_TESLA) !=
        this->dataPtr->noises.end())
    {
      field.X(
          this->dataPtr->noises[MAGNETOMETER_X_NOISE_TESLA]->Apply(field.X()));
    }

    if (this->dataPtr->noises.find(MAGNETOMETER_Y_NOISE_TESLA) !=
        this->dataPtr->noises.end())
    {
      field.Y(
          this->dataPtr->noises[MAGNETOMETER_Y_NOISE_TESLA]->Apply(field.Y()));
    }

    if (this->dataPtr->noises.find(MAGNETOMETER_Z_NOISE_TESLA) !=
        this->dataPtr->noises.end())
    {
      field.Z(
          this->dataPtr->noises[MAGNETOMETER_Z_NOISE_TESLA]->Apply(field.Z()));
    }

    // Set the body-frame magnetic field strength
    msgs::Set(this->dataPtr->magMsg.mutable_field_tesla(), field);
  }

  // Save the time of the measurement
  msgs::Set(this->dataPtr->magMsg.mutable_time(),
            this->dataPtr->world->GetSimTime());

  // Publish the message if needed
  if (this->dataPtr->magPub)
    this->dataPtr->magPub->Publish(this->dataPtr->magMsg);

  return true;
}

//////////////////////////////////////////////////
ignition::math::Vector3d MagnetometerSensor::MagneticField() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return msgs::ConvertIgn(this->dataPtr->magMsg.field_tesla());
}
