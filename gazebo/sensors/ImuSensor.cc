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

#include <boost/algorithm/string.hpp>
#include <ignition/math/Rand.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ImuSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor)

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
  : Sensor(sensors::OTHER)
{
  this->dataIndex = 0;
  this->dataDirty = false;
  this->incomingLinkData[0].reset();
  this->incomingLinkData[1].reset();
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/imu";
  boost::replace_all(topicName, "::", "/");

  this->pub = this->node->Advertise<msgs::IMU>(topicName, 500);

  // Get the imu element pointer
  sdf::ElementPtr imuElem = this->sdf->GetElement("imu");

  // If an angular velocity noise models have been specified, create them
  if (imuElem->HasElement("angular_velocity"))
  {
    std::ostringstream out;

    out << "Applying angular velocity noise to IMU["
      << this->GetName() << "].\n";

    sdf::ElementPtr angularElem = imuElem->GetElement("angular_velocity");

    if (angularElem->HasElement("x") &&
        angularElem->GetElement("x")->HasElement("noise"))
    {
      this->noises[IMU_ANGVEL_X_NOISE_RADIANS_PER_S] =
        NoiseFactory::NewNoiseModel(
            angularElem->GetElement("x")->GetElement("noise"));

      out << "  X: ";
      this->noises[IMU_ANGVEL_X_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    if (angularElem->HasElement("y") &&
        angularElem->GetElement("y")->HasElement("noise"))
    {
      this->noises[IMU_ANGVEL_Y_NOISE_RADIANS_PER_S] =
        NoiseFactory::NewNoiseModel(
            angularElem->GetElement("y")->GetElement("noise"));

      out << "  Y: ";
      this->noises[IMU_ANGVEL_Y_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    if (angularElem->HasElement("z") &&
        angularElem->GetElement("z")->HasElement("noise"))
    {
      this->noises[IMU_ANGVEL_Z_NOISE_RADIANS_PER_S] =
        NoiseFactory::NewNoiseModel(
            angularElem->GetElement("z")->GetElement("noise"));

      out << "  Z: ";
      this->noises[IMU_ANGVEL_Z_NOISE_RADIANS_PER_S]->Print(out);
      out << std::endl;
    }

    gzlog << out.str();
  }

  // If linear acceleration noise models have been specified, create them
  if (imuElem->HasElement("linear_acceleration"))
  {
    std::ostringstream out;
    out << "Applying linear acceleration noise to IMU["
      << this->GetName() << "].\n";

    sdf::ElementPtr linearElem = imuElem->GetElement("linear_acceleration");
    if (linearElem->HasElement("x") &&
        linearElem->GetElement("x")->HasElement("noise"))
    {
      this->noises[IMU_LINACC_X_NOISE_METERS_PER_S_SQR] =
        NoiseFactory::NewNoiseModel(
            linearElem->GetElement("x")->GetElement("noise"));

      out << "  X: ";
      this->noises[IMU_LINACC_X_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    if (linearElem->HasElement("y") &&
        linearElem->GetElement("y")->HasElement("noise"))
    {
      this->noises[IMU_LINACC_Y_NOISE_METERS_PER_S_SQR] =
        NoiseFactory::NewNoiseModel(
            linearElem->GetElement("y")->GetElement("noise"));

      out << "  Y: ";
      this->noises[IMU_LINACC_Y_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    if (linearElem->HasElement("z") &&
        linearElem->GetElement("z")->HasElement("noise"))
    {
      this->noises[IMU_LINACC_Z_NOISE_METERS_PER_S_SQR] =
        NoiseFactory::NewNoiseModel(
            linearElem->GetElement("z")->GetElement("noise"));

      out << "  Z: ";
      this->noises[IMU_LINACC_Z_NOISE_METERS_PER_S_SQR]->Print(out);
      out << std::endl;
    }

    gzlog << out.str();
  }

  // Start publishing measurements on the topic.
  this->parentEntity->SetPublishData(true);

  std::string topic = "~/" + this->parentEntity->GetScopedName();
  this->linkDataSub = this->node->Subscribe(topic,
    &ImuSensor::OnLinkData, this);
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(this->parentName));

  if (!this->parentEntity)
  {
    gzthrow("IMU has invalid parent[" + this->parentName +
            "]. Must be a link\n");
  }
  this->referencePose = this->pose + this->parentEntity->GetWorldPose().Ign();
  this->lastLinearVel = this->referencePose.Rot().RotateVector(
    this->parentEntity->GetWorldLinearVel().Ign());
}

//////////////////////////////////////////////////
void ImuSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void ImuSensor::Fini()
{
  this->parentEntity->SetPublishData(false);
  this->pub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
msgs::IMU ImuSensor::GetImuMessage() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->imuMsg;
}

//////////////////////////////////////////////////
void ImuSensor::OnLinkData(ConstLinkDataPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  // Store the contacts message for processing in UpdateImpl
  this->incomingLinkData[this->dataIndex] = _msg;
  this->dataDirty = true;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ImuSensor::AngularVelocity(const bool _noiseFree) const
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (_noiseFree)
    return this->angularVel;
  return msgs::ConvertIgn(this->imuMsg.angular_velocity());
}

//////////////////////////////////////////////////
ignition::math::Vector3d ImuSensor::LinearAcceleration(
    const bool _noiseFree) const
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (_noiseFree)
    return this->linearAcc;
  return msgs::ConvertIgn(this->imuMsg.linear_acceleration());
}

//////////////////////////////////////////////////
ignition::math::Quaterniond ImuSensor::Orientation() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::ConvertIgn(this->imuMsg.orientation());
}

//////////////////////////////////////////////////
void ImuSensor::SetReferencePose()
{
  this->referencePose = this->pose + this->parentEntity->GetWorldPose().Ign();
}

//////////////////////////////////////////////////
bool ImuSensor::UpdateImpl(bool /*_force*/)
{
  msgs::LinkData msg;
  int readIndex = 0;

  {
    boost::mutex::scoped_lock lock(this->mutex);

    // Don't do anything if there is no new data to process.
    if (!this->dataDirty)
      return false;

    readIndex = this->dataIndex;
    this->dataIndex ^= 1;
    this->dataDirty = false;
  }

  // toggle the index
  msg.CopyFrom(*this->incomingLinkData[readIndex].get());

  common::Time timestamp = msgs::Convert(msg.time());

  double dt = (timestamp - this->lastMeasurementTime).Double();

  this->lastMeasurementTime = timestamp;

  if (dt > 0.0)
  {
    boost::mutex::scoped_lock lock(this->mutex);

    this->imuMsg.set_entity_name(this->parentName);

    this->gravity = this->world->GetPhysicsEngine()->GetGravity().Ign();

    msgs::Set(this->imuMsg.mutable_stamp(), timestamp);

    ignition::math::Pose3d parentEntityPose =
      this->parentEntity->GetWorldPose().Ign();
    ignition::math::Pose3d imuPose = this->pose + parentEntityPose;

    // Get the angular velocity
    ignition::math::Vector3d imuWorldAngularVel = msgs::ConvertIgn(
        msg.angular_velocity());

    // Set the IMU angular velocity
    this->angularVel = imuPose.Rot().Inverse().RotateVector(
        imuWorldAngularVel);
    msgs::Set(this->imuMsg.mutable_angular_velocity(), this->angularVel);

    // Compute and set the IMU linear acceleration
    ignition::math::Vector3d imuWorldLinearVel
        = msgs::ConvertIgn(msg.linear_velocity());
    // Get the correct vel for imu's that are at an offset from parent link
    imuWorldLinearVel +=
        imuWorldAngularVel.Cross(parentEntityPose.Pos() - imuPose.Pos());
    this->linearAcc = imuPose.Rot().Inverse().RotateVector(
      (imuWorldLinearVel - this->lastLinearVel) / dt);

    // Add contribution from gravity
    this->linearAcc -= imuPose.Rot().Inverse().RotateVector(this->gravity);
    msgs::Set(this->imuMsg.mutable_linear_acceleration(), this->linearAcc);

    // Set the IMU orientation
    msgs::Set(this->imuMsg.mutable_orientation(),
              (imuPose - this->referencePose).Rot());

    this->lastLinearVel = imuWorldLinearVel;

    // Apply noise models
    for (auto const &keyNoise : this->noises)
    {
      switch (keyNoise.first)
      {
        case IMU_ANGVEL_X_NOISE_RADIANS_PER_S:
          this->imuMsg.mutable_angular_velocity()->set_x(
            keyNoise.second->Apply(this->imuMsg.angular_velocity().x()));
          break;
        case IMU_ANGVEL_Y_NOISE_RADIANS_PER_S:
          this->imuMsg.mutable_angular_velocity()->set_y(
            keyNoise.second->Apply(this->imuMsg.angular_velocity().y()));
          break;
        case IMU_ANGVEL_Z_NOISE_RADIANS_PER_S:
          this->imuMsg.mutable_angular_velocity()->set_z(
            keyNoise.second->Apply(this->imuMsg.angular_velocity().z()));
          break;
        case IMU_LINACC_X_NOISE_METERS_PER_S_SQR:
          this->imuMsg.mutable_linear_acceleration()->set_x(
            keyNoise.second->Apply(this->imuMsg.linear_acceleration().x()));
          break;
        case IMU_LINACC_Y_NOISE_METERS_PER_S_SQR:
          this->imuMsg.mutable_linear_acceleration()->set_y(
            keyNoise.second->Apply(this->imuMsg.linear_acceleration().y()));
          break;
        case IMU_LINACC_Z_NOISE_METERS_PER_S_SQR:
          this->imuMsg.mutable_linear_acceleration()->set_z(
            keyNoise.second->Apply(this->imuMsg.linear_acceleration().z()));
          break;
        default:
          std::ostringstream out;
          out << "Removing unrecognized noise model: ";
          keyNoise.second->Print(out);
          out << std::endl;
          gzwarn << out.str() << std::endl;
          this->noises.erase(keyNoise.first);
          break;
      }
    }

    // Publish the message
    if (this->pub)
      this->pub->Publish(this->imuMsg);
  }

  return true;
}

//////////////////////////////////////////////////
bool ImuSensor::IsActive()
{
  return this->active ||
         (this->pub && this->pub->HasConnections());
}
