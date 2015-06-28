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

#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/OrientationSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("orientation", OrientationSensor)

/////////////////////////////////////////////////
OrientationSensor::OrientationSensor()
: Sensor(sensors::OTHER)
{
}

/////////////////////////////////////////////////
OrientationSensor::~OrientationSensor()
{
}

/////////////////////////////////////////////////
void OrientationSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void OrientationSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->parentLink = boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  this->topicName = "~/" + this->parentName + '/' + this->GetName();
  if (this->sdf->HasElement("topic"))
    this->topicName += '/' + this->sdf->Get<std::string>("topic");
  boost::replace_all(this->topicName, "::", "/");

  this->orientPub = this->node->Advertise<msgs::Orientation>(
    this->topicName, 50);

  // Parse sdf noise parameters
  sdf::ElementPtr orientElem = this->sdf->GetElement("orientation");

  // Load magnetic field noise parameters
  {
    this->noises[ORIENTATION_X_NOISE_RADIANS] = NoiseFactory::NewNoiseModel(
      orientElem->GetElement("x")->GetElement("noise"));
    this->noises[ORIENTATION_Y_NOISE_RADIANS] = NoiseFactory::NewNoiseModel(
      orientElem->GetElement("y")->GetElement("noise"));
    this->noises[ORIENTATION_Z_NOISE_RADIANS] = NoiseFactory::NewNoiseModel(
      orientElem->GetElement("z")->GetElement("noise"));
  }
}

/////////////////////////////////////////////////
void OrientationSensor::Fini()
{
  Sensor::Fini();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
void OrientationSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool OrientationSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);
  
  // Get latest pose information
  if (this->parentLink)
  {
    // Get pose in gazebo reference frame
    math::Pose pose = this->pose + this->parentLink->GetWorldPose();

    // Extract just the rotation as an euler angle
    math::Vector3 rot = pose.rot.GetAsEuler();

    // Apply orientation noise to each axis independently
    rot.x = this->noises[ORIENTATION_X_NOISE_RADIANS]->Apply(rot.x);
    rot.y = this->noises[ORIENTATION_Y_NOISE_RADIANS]->Apply(rot.y);
    rot.z = this->noises[ORIENTATION_Z_NOISE_RADIANS]->Apply(rot.z);

    // Set the euler and quaternion values for the orientation
    msgs::Set(this->orientMsg.mutable_euler(),rot);
    msgs::Set(this->orientMsg.mutable_quaternion(),
      math::Quaternion::EulerToQuaternion(rot));
  }

  // Save the time of the measurement
  msgs::Set(this->orientMsg.mutable_time(), this->world->GetSimTime());

  // Publish the message if needed
  if (this->orientPub)
    this->orientPub->Publish(this->orientMsg);

  return true;
}

//////////////////////////////////////////////////
math::Vector3 OrientationSensor::GetEulerOrientation() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->orientMsg.euler());
}

//////////////////////////////////////////////////
math::Quaternion OrientationSensor::GetQuaternionOrientation() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->orientMsg.quaternion());
}