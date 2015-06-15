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
/* Desc: IMU sensor
 * Author: Matt Thompson
 * Date: 6 September 2008
*/


#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/noise/Noise.hh"
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

  if (this->sdf->HasElement("imu") &&
      this->sdf->GetElement("imu")->HasElement("topic") &&
      this->sdf->GetElement("imu")->Get<std::string>("topic")
      != "__default_topic__")
  {
    this->pub = this->node->Advertise<msgs::IMU>(
        this->sdf->GetElement("imu")->Get<std::string>("topic"), 500);
  }
  else
  {
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/imu";
    boost::replace_all(topicName, "::", "/");

    this->pub = this->node->Advertise<msgs::IMU>(topicName, 500);
  }

  // Parse sdf noise parameters
  sdf::ElementPtr imuElem = this->sdf->GetElement("imu");

  // Load angular velocity noise parameters
  {
    sdf::ElementPtr fieldElem = magElem->GetElement("angular_velocity");
    this->noises[AngVelNoiseX] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_x")->GetElement("noise"));
    this->noises[AngVelNoiseY] =  NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_y")->GetElement("noise"));
    this->noises[AngVelNoiseZ] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_z")->GetElement("noise"));
  }

  // Load lienar acceleration noise parameters
  {
    sdf::ElementPtr fieldElem = magElem->GetElement("linear_acceleration");
    this->noises[LinAccNoiseX] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_x")->GetElement("noise"));
    this->noises[LinAccNoiseY] =  NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_y")->GetElement("noise"));
    this->noises[LinAccNoiseZ] = NoiseFactory::NewNoiseModel(
      fieldElem->GetElement("body_z")->GetElement("noise"));
  }

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
  this->referencePose = this->pose + this->parentEntity->GetWorldPose();
  this->lastLinearVel = this->referencePose.rot.RotateVector(
    this->parentEntity->GetWorldLinearVel());
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
math::Vector3 ImuSensor::GetAngularVelocity() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->imuMsg.angular_velocity());
}

//////////////////////////////////////////////////
math::Vector3 ImuSensor::GetLinearAcceleration() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->imuMsg.linear_acceleration());
}

//////////////////////////////////////////////////
math::Quaternion ImuSensor::GetOrientation() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return msgs::Convert(this->imuMsg.orientation());
}

//////////////////////////////////////////////////
void ImuSensor::SetReferencePose()
{
  this->referencePose = this->pose + this->parentEntity->GetWorldPose();
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

    this->gravity = this->world->GetPhysicsEngine()->GetGravity();

    msgs::Set(this->imuMsg.mutable_stamp(), timestamp);

    math::Pose parentEntityPose = this->parentEntity->GetWorldPose();
    math::Pose imuPose = this->pose + parentEntityPose;

    // Set the IMU angular velocity
    math::Vector3 imuWorldAngularVel
        = msgs::Convert(msg.angular_velocity());

    msgs::Set(this->imuMsg.mutable_angular_velocity(),
              imuPose.rot.GetInverse().RotateVector(
              imuWorldAngularVel));

    // Compute and set the IMU linear acceleration
    math::Vector3 imuWorldLinearVel
        = msgs::Convert(msg.linear_velocity());

    // Get the correct vel for imu's that are at an offset from parent link
    imuWorldLinearVel +=
        imuWorldAngularVel.Cross(parentEntityPose.pos - imuPose.pos);
    this->linearAcc = imuPose.rot.GetInverse().RotateVector(
      (imuWorldLinearVel - this->lastLinearVel) / dt);

    // Add contribution from gravity
    this->linearAcc -= imuPose.rot.GetInverse().RotateVector(this->gravity);
    msgs::Set(this->imuMsg.mutable_linear_acceleration(), this->linearAcc);

    // Set the IMU orientation
    msgs::Set(this->imuMsg.mutable_orientation(),
              (imuPose - this->referencePose).rot);
    this->lastLinearVel = imuWorldLinearVel;

    // Apply angular velocity noise
    this->imuMsg.mutable_angular_velocity()->set_x(
      this->noises[AngVelNoiseX].Apply(this->imuMsg.angular_velocity().x()));
    this->imuMsg.mutable_angular_velocity()->set_y(
      this->noises[AngVelNoiseY].Apply(this->imuMsg.angular_velocity().y()));
    this->imuMsg.mutable_angular_velocity()->set_z(
      this->noises[AngVelNoiseZ].Apply(this->imuMsg.angular_velocity().z()));

    // Apply linear acceleration noise
    this->imuMsg.mutable_linear_acceleration()->set_x(
      this->noises[AngVelNoiseX].Apply(this->imuMsg.linear_acceleration().x()));
    this->imuMsg.mutable_linear_acceleration()->set_y(
      this->noises[AngVelNoiseY].Apply(this->imuMsg.linear_acceleration().y()));
    this->imuMsg.mutable_linear_acceleration()->set_z(
      this->noises[AngVelNoiseZ].Apply(this->imuMsg.linear_acceleration().z()));

    // Publish if needed
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
