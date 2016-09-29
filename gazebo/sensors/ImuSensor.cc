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

#include <boost/algorithm/string.hpp>
#include <ignition/math/Rand.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ImuSensorPrivate.hh"
#include "gazebo/sensors/ImuSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor)

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
: Sensor(sensors::OTHER),
  dataPtr(new ImuSensorPrivate)
{
  this->dataPtr->dataIndex = 0;
  this->dataPtr->dataDirty = false;
  this->dataPtr->incomingLinkData[0].reset();
  this->dataPtr->incomingLinkData[1].reset();
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  // Initialize orientation to identity.
  msgs::Set(this->dataPtr->imuMsg.mutable_orientation(),
      ignition::math::Quaterniond::Identity);

  // initialize worldToReference transform as local frame
  this->dataPtr->worldToReference = (this->pose +
    this->dataPtr->parentEntity->GetWorldPose().Ign()).Rot();

  // CASE 1 : Topic is specified in the sensor itself (should be deprecated!)
  if (this->sdf->HasElement("imu") &&
      this->sdf->GetElement("imu")->HasElement("topic") &&
      this->sdf->GetElement("imu")->Get<std::string>("topic")
      != "__default_topic__")
  {
    this->dataPtr->pub = this->node->Advertise<msgs::IMU>(
        this->sdf->GetElement("imu")->Get<std::string>("topic"), 500);
  }
  // CASE 2 : Topic is specified in parent sensor definition
  else
  {
    std::string topicName = "~/";
    topicName += this->ParentName() + "/" + this->Name() + "/imu";
    boost::replace_all(topicName, "::", "/");

    this->dataPtr->pub =
      this->node->Advertise<msgs::IMU>(topicName, 500);
  }

  // Get the imu element pointer
  sdf::ElementPtr imuElem = this->sdf->GetElement("imu");

  // If an angular velocity noise models have been specified, create them
  if (imuElem->HasElement("angular_velocity"))
  {
    std::ostringstream out;

    out << "Applying angular velocity noise to IMU["
      << this->Name() << "].\n";

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
      << this->Name() << "].\n";

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
  this->dataPtr->parentEntity->SetPublishData(true);

  std::string topic = "~/" + this->dataPtr->parentEntity->GetScopedName();
  this->dataPtr->linkDataSub = this->node->Subscribe(topic,
    &ImuSensor::OnLinkData, this);
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->dataPtr->parentEntity = boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(this->ParentName()));

  if (!this->dataPtr->parentEntity)
  {
    gzthrow("IMU has invalid parent[" + this->ParentName() +
            "]. Must be a link\n");
  }

  /////////////////////////////////////////////////////////////////
  // compute the last linear velocity of the imu in the world frame
  // for computing acceleration based on finite differencing
  /////////////////////////////////////////////////////////////////
  // next, account for vel in world frame of the imu
  // given the imu frame is offset from link frame, and link is rotating
  this->dataPtr->lastImuWorldLinearVel =
      this->dataPtr->parentEntity->GetWorldLinearVel(this->pose.Pos()).Ign();
}

//////////////////////////////////////////////////
void ImuSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void ImuSensor::Fini()
{
  // Clean transport
  {
    this->dataPtr->pub.reset();
    this->dataPtr->linkDataSub.reset();
  }

  if (this->dataPtr->parentEntity)
    this->dataPtr->parentEntity->SetPublishData(false);
  this->dataPtr->parentEntity.reset();

  this->dataPtr->incomingLinkData[0].reset();
  this->dataPtr->incomingLinkData[1].reset();

  Sensor::Fini();
}

//////////////////////////////////////////////////
msgs::IMU ImuSensor::ImuMessage() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->imuMsg;
}

//////////////////////////////////////////////////
void ImuSensor::OnLinkData(ConstLinkDataPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // Store the contacts message for processing in UpdateImpl
  this->dataPtr->incomingLinkData[this->dataPtr->dataIndex] = _msg;
  this->dataPtr->dataDirty = true;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ImuSensor::AngularVelocity(const bool _noiseFree) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_noiseFree)
    return this->dataPtr->angularVel;
  return msgs::ConvertIgn(this->dataPtr->imuMsg.angular_velocity());
}

//////////////////////////////////////////////////
ignition::math::Vector3d ImuSensor::LinearAcceleration(
    const bool _noiseFree) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_noiseFree)
    return this->dataPtr->linearAcc;
  return msgs::ConvertIgn(this->dataPtr->imuMsg.linear_acceleration());
}

//////////////////////////////////////////////////
ignition::math::Quaterniond ImuSensor::Orientation() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return msgs::ConvertIgn(this->dataPtr->imuMsg.orientation());
}

//////////////////////////////////////////////////
void ImuSensor::SetReferencePose()
{
  // this call sets the current imu pose as the imu's reference pose
  this->SetWorldToReferenceOrientation(
      (this->pose + this->dataPtr->parentEntity->GetWorldPose().Ign()).Rot());
}

//////////////////////////////////////////////////
void ImuSensor::SetWorldToReferencePose(
  const ignition::math::Pose3d &_pose)
{
  // worldToReference: from world frame to IMU Reference frame.
  gzwarn << "translation part of the argument is not used."
         << " Use SetWorldToReferenceOrientation instead.\n";
  this->dataPtr->worldToReference = _pose.Rot();
}

//////////////////////////////////////////////////
void ImuSensor::SetWorldToReferenceOrientation(
  const ignition::math::Quaterniond &_orientation)
{
  this->dataPtr->worldToReference = _orientation;
}

//////////////////////////////////////////////////
bool ImuSensor::UpdateImpl(const bool /*_force*/)
{
  msgs::LinkData msg;
  int readIndex = 0;

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    // Don't do anything if there is no new data to process.
    if (!this->dataPtr->dataDirty)
      return false;

    readIndex = this->dataPtr->dataIndex;
    this->dataPtr->dataIndex ^= 1;
    this->dataPtr->dataDirty = false;
  }

  // toggle the index
  msg.CopyFrom(*this->dataPtr->incomingLinkData[readIndex].get());

  common::Time timestamp = msgs::Convert(msg.time());

  double dt = (timestamp - this->lastMeasurementTime).Double();

  this->lastMeasurementTime = timestamp;

  if (dt > 0.0)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    this->dataPtr->imuMsg.set_entity_name(this->ParentName());

    this->dataPtr->gravity = this->world->Gravity();

    msgs::Set(this->dataPtr->imuMsg.mutable_stamp(), timestamp);

    ignition::math::Pose3d parentEntityPose =
      this->dataPtr->parentEntity->GetWorldPose().Ign();
    ignition::math::Pose3d imuWorldPose = this->pose + parentEntityPose;

    // Get the angular velocity
    ignition::math::Vector3d linkWorldAngularVel = msgs::ConvertIgn(
        msg.angular_velocity());

    /////////////////////////////////////////////////////////////////////
    // Set the IMU angular velocity (defined in imu's local frame)
    /////////////////////////////////////////////////////////////////////
    this->dataPtr->angularVel = imuWorldPose.Rot().Inverse().RotateVector(
        linkWorldAngularVel);
    msgs::Set(this->dataPtr->imuMsg.mutable_angular_velocity(),
        this->dataPtr->angularVel);

    /////////////////////////////////////////////////////////////////////
    // Compute and set the IMU linear acceleration in the imu local frame
    /////////////////////////////////////////////////////////////////////
    // first get imu link's linear velocity in world frame
    ignition::math::Vector3d linkWorldLinearVel
        = msgs::ConvertIgn(msg.linear_velocity());
    // next, account for vel in world frame of the imu
    // given the imu frame is offset from link frame, and link is rotating
    // compute the velocity of the imu axis origin in world frame
    ignition::math::Vector3d imuWorldLinearVel = linkWorldLinearVel +
        linkWorldAngularVel.Cross(imuWorldPose.Pos() - parentEntityPose.Pos());
    // compute acceleration by differentiating velocity in world frame,
    // and rotate into imu local frame
    this->dataPtr->linearAcc = imuWorldPose.Rot().Inverse().RotateVector(
      (imuWorldLinearVel - this->dataPtr->lastImuWorldLinearVel) / dt);

    // Add contribution from gravity
    // Do we want to skip if link does not have gravity enabled?
    //   e.g. if (this->dataPtr->parentEntity->GetGravityMode())
    this->dataPtr->linearAcc -= imuWorldPose.Rot().Inverse().RotateVector(
        this->dataPtr->gravity);

    // publish linear acceleration
    msgs::Set(this->dataPtr->imuMsg.mutable_linear_acceleration(),
        this->dataPtr->linearAcc);

    // Set the IMU orientation
    // imu orientation with respect to reference frame
    ignition::math::Quaterniond imuReferenceOrientation =
      (this->dataPtr->worldToReference.Inverse() * imuWorldPose.Rot());
    msgs::Set(this->dataPtr->imuMsg.mutable_orientation(),
      imuReferenceOrientation);

    this->dataPtr->lastImuWorldLinearVel = imuWorldLinearVel;

    // Apply noise models
    for (auto const &keyNoise : this->noises)
    {
      switch (keyNoise.first)
      {
        case IMU_ANGVEL_X_NOISE_RADIANS_PER_S:
          this->dataPtr->imuMsg.mutable_angular_velocity()->set_x(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.angular_velocity().x()));
          break;
        case IMU_ANGVEL_Y_NOISE_RADIANS_PER_S:
          this->dataPtr->imuMsg.mutable_angular_velocity()->set_y(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.angular_velocity().y()));
          break;
        case IMU_ANGVEL_Z_NOISE_RADIANS_PER_S:
          this->dataPtr->imuMsg.mutable_angular_velocity()->set_z(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.angular_velocity().z()));
          break;
        case IMU_LINACC_X_NOISE_METERS_PER_S_SQR:
          this->dataPtr->imuMsg.mutable_linear_acceleration()->set_x(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.linear_acceleration().x()));
          break;
        case IMU_LINACC_Y_NOISE_METERS_PER_S_SQR:
          this->dataPtr->imuMsg.mutable_linear_acceleration()->set_y(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.linear_acceleration().y()));
          break;
        case IMU_LINACC_Z_NOISE_METERS_PER_S_SQR:
          this->dataPtr->imuMsg.mutable_linear_acceleration()->set_z(
            keyNoise.second->Apply(
              this->dataPtr->imuMsg.linear_acceleration().z()));
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
    if (this->dataPtr->pub)
      this->dataPtr->pub->Publish(this->dataPtr->imuMsg);
  }

  return true;
}

//////////////////////////////////////////////////
bool ImuSensor::IsActive() const
{
  return this->active ||
         (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
}
