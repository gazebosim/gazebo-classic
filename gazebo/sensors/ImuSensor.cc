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

  // CASE 1 : Topic is specified in the sensor itself (should be deprecated!)
  if (this->sdf->HasElement("imu") &&
      this->sdf->GetElement("imu")->HasElement("topic") &&
      this->sdf->GetElement("imu")->Get<std::string>("topic")
      != "__default_topic__")
  {
    this->pub = this->node->Advertise<msgs::IMU>(
        this->sdf->GetElement("imu")->Get<std::string>("topic"), 500);
  }
  // CASE 2 : Topic is specified in parent sensor definition
  else
  {
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/imu";
    boost::replace_all(topicName, "::", "/");
    this->pub = this->node->Advertise<msgs::IMU>(topicName, 500);
  }

  // Get the imu element pointer
  sdf::ElementPtr imuElem = this->sdf->GetElement("imu");

  // CASE 1 : Noise is defined within the sensor (should be deprecated)
  if (imuElem->HasElement("noise"))
  {
    sdf::ElementPtr noiseElem = imuElem->GetElement("noise");
    std::string type = noiseElem->Get<std::string>("type");
    if (type == "gaussian")
    {
      if (noiseElem->HasElement("rate"))
      {
        sdf::ElementPtr rateElem = noiseElem->GetElement("rate");

        // Rename rate -> noise to enforce forward compatibility
        rateElem->SetName("noise");
        rateElem->AddAttribute("type","string","gaussian",true);

        // Create the noise streams
        this->noises[IMU_ANGVEL_X_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rateElem);
        this->noises[IMU_ANGVEL_Y_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rateElem);
        this->noises[IMU_ANGVEL_Z_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rateElem);

        // Rename noise -> rate to enforce forward compatibility
        rateElem->SetName("rate");
      }

      if (noiseElem->HasElement("accel"))
      {
        sdf::ElementPtr accelElem = noiseElem->GetElement("accel");

        // Rename accel -> noise to enforce forward compatibility
        accelElem->SetName("noise");
        accelElem->AddAttribute("type","string","gaussian",true);

        // Create the noise streams
        this->noises[IMU_LINACC_X_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(accelElem);
        this->noises[IMU_LINACC_Y_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(accelElem);
        this->noises[IMU_LINACC_Z_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(accelElem);

        // Rename noise -> accel to enforce forward compatibility
        accelElem->SetName("accel");
      }
    }
    else
      gzwarn << "ignoring unknown noise model type \"" << type << "\"" <<
        std::endl;
  }
  // CASE 2: noise specified in using newer generic SDF
  else
  {

    // If a linear acceleration noise model has been specified, create it 
    if (imuElem->HasElement("linear_acceleration"))
    {
      sdf::ElementPtr rE = noiseElem->GetElement("linear_acceleration");
      if (accelElem->HasElement("x"))
        this->noises[IMU_ANGVEL_X_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rE->GetElement("x")->GetElement("noise"));
      if (accelElem->HasElement("y"))
        this->noises[IMU_ANGVEL_Y_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rE->GetElement("y")->GetElement("noise"));
      if (accelElem->HasElement("z"))
        this->noises[IMU_ANGVEL_Z_NOISE_RAD_PER_S] = 
          NoiseFactory::NewNoiseModel(rE->GetElement("z")->GetElement("noise"));
    }

    // If a linear acceleration noise model has been specified, create it 
    if (imuElem->HasElement("linear_acceleration"))
    {
      sdf::ElementPtr aE = noiseElem->GetElement("linear_acceleration");
      if (accelElem->HasElement("x"))
        this->noises[IMU_LINACC_X_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(aE->GetElement("x")->GetElement("noise"));
      if (accelElem->HasElement("y"))
        this->noises[IMU_LINACC_Y_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(aE->GetElement("y")->GetElement("noise"));
      if (accelElem->HasElement("z"))
        this->noises[IMU_LINACC_Z_NOISE_METERS_PER_S_SQR] = 
          NoiseFactory::NewNoiseModel(aE->GetElement("z")->GetElement("noise"));
    }
  }

  // Start publishing data on the topic
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

    // Perturb the angular velocity
    this->imuMsg.mutable_angular_velocity()->set_x(
      this->noises[IMU_ANGVEL_X_NOISE_RAD_PER_S]->Apply(
        this->imuMsg.angular_velocity().x()));
    this->imuMsg.mutable_angular_velocity()->set_y(
      this->noises[IMU_ANGVEL_Y_NOISE_RAD_PER_S]->Apply(
        this->imuMsg.angular_velocity().y()));
    this->imuMsg.mutable_angular_velocity()->set_z(
      this->noises[IMU_ANGVEL_Z_NOISE_RAD_PER_S]->Apply(
        this->imuMsg.angular_velocity().z()));

    // Perturb the linear acceleration
    this->imuMsg.mutable_linear_acceleration()->set_x(
      this->noises[IMU_LINACC_X_NOISE_METERS_PER_S_SQR]->Apply(
        this->imuMsg.linear_acceleration().x()));
    this->imuMsg.mutable_linear_acceleration()->set_y(
      this->noises[IMU_LINACC_Y_NOISE_METERS_PER_S_SQR]->Apply(
        this->imuMsg.linear_acceleration().y()));
    this->imuMsg.mutable_linear_acceleration()->set_z(
      this->noises[IMU_LINACC_Z_NOISE_METERS_PER_S_SQR]->Apply(
        this->imuMsg.linear_acceleration().z()));

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
