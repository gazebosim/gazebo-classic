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
#include <boost/algorithm/string.hpp>


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

  // Handle noise model settings.
  this->noiseActive = false;
  sdf::ElementPtr imuElem = this->sdf->GetElement("imu");
  if (imuElem->HasElement("noise"))
  {
    sdf::ElementPtr noiseElem = imuElem->GetElement("noise");
    std::string type = noiseElem->Get<std::string>("type");
    if (type == "gaussian")
    {
      this->noiseActive = true;
      this->noiseType = GAUSSIAN;
      this->rateNoiseMean = 0.0;
      this->rateNoiseStdDev = 0.0;
      this->rateBias = 0.0;
      this->accelNoiseMean = 0.0;
      this->accelNoiseStdDev = 0.0;
      this->accelBias = 0.0;
      if (noiseElem->HasElement("rate"))
      {
        sdf::ElementPtr rateElem = noiseElem->GetElement("rate");
        this->rateNoiseMean = rateElem->Get<double>("mean");
        this->rateNoiseStdDev = rateElem->Get<double>("stddev");
        double rateBiasMean = rateElem->Get<double>("bias_mean");
        double rateBiasStddev = rateElem->Get<double>("bias_stddev");
        // Sample the bias that we'll use later
        this->rateBias = math::Rand::GetDblNormal(rateBiasMean, rateBiasStddev);
        // With equal probability, we pick a negative bias (by convention,
        // rateBiasMean should be positive, though it would work fine if
        // negative).
        if (math::Rand::GetDblUniform() < 0.5)
          this->rateBias = -this->rateBias;
        gzlog << "applying Gaussian noise model to rate with mean " <<
          this->rateNoiseMean << " and stddev " << this->rateNoiseStdDev <<
          ", bias " << this->rateBias << std::endl;
      }
      if (noiseElem->HasElement("accel"))
      {
        sdf::ElementPtr accelElem = noiseElem->GetElement("accel");
        this->accelNoiseMean = accelElem->Get<double>("mean");
        this->accelNoiseStdDev = accelElem->Get<double>("stddev");
        double accelBiasMean = accelElem->Get<double>("bias_mean");
        double accelBiasStddev = accelElem->Get<double>("bias_stddev");
        // Sample the bias that we'll use later
        this->accelBias = math::Rand::GetDblNormal(accelBiasMean,
                                                   accelBiasStddev);
        // With equal probability, we pick a negative bias (by convention,
        // accelBiasMean should be positive, though it would work fine if
        // negative).
        if (math::Rand::GetDblUniform() < 0.5)
          this->accelBias = -this->accelBias;
        gzlog << "applying Gaussian noise model to accel with mean " <<
          this->accelNoiseMean << " and stddev " << this->accelNoiseStdDev <<
          ", bias " << this->accelBias << std::endl;
      }
    }
    else
      gzwarn << "ignoring unknown noise model type \"" << type << "\"" <<
        std::endl;
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

    if (this->noiseActive)
    {
      switch (this->noiseType)
      {
        case GAUSSIAN:
          // Add Gaussian noise + fixed bias to each rate
          this->imuMsg.mutable_angular_velocity()->set_x(
            this->imuMsg.angular_velocity().x() + this->rateBias +
            math::Rand::GetDblNormal(this->rateNoiseMean,
              this->rateNoiseStdDev));
          this->imuMsg.mutable_angular_velocity()->set_y(
            this->imuMsg.angular_velocity().y() + this->rateBias +
            math::Rand::GetDblNormal(this->rateNoiseMean,
              this->rateNoiseStdDev));
          this->imuMsg.mutable_angular_velocity()->set_z(
            this->imuMsg.angular_velocity().z() + this->rateBias +
            math::Rand::GetDblNormal(this->rateNoiseMean,
              this->rateNoiseStdDev));

          // Add Gaussian noise + fixed bias to each acceleration
          this->imuMsg.mutable_linear_acceleration()->set_x(
            this->imuMsg.linear_acceleration().x() + this->accelBias +
            math::Rand::GetDblNormal(this->accelNoiseMean,
                                     this->accelNoiseStdDev));
          this->imuMsg.mutable_linear_acceleration()->set_y(
            this->imuMsg.linear_acceleration().y() + this->accelBias +
            math::Rand::GetDblNormal(this->accelNoiseMean,
                                     this->accelNoiseStdDev));
          this->imuMsg.mutable_linear_acceleration()->set_z(
            this->imuMsg.linear_acceleration().z() + this->accelBias +
            math::Rand::GetDblNormal(this->accelNoiseMean,
                                     this->accelNoiseStdDev));

          // TODO: add noise to orientation
          break;
        default:
          GZ_ASSERT(false, "Invalid noise model type");
      }
    }

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
