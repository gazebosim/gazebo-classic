/*
 * Copyright 2012 Open Source Robotics Foundation
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


#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"

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
    : Sensor()
{
  std::cout << "NEW IMU\n";
  this->linearAcc = math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  // this->sdf->PrintValues("  ");

  if (this->sdf->HasElement("imu") &&
      this->sdf->GetElement("imu")->HasElement("topic") &&
      this->sdf->GetElement("imu")->GetValueString("topic")
      != "__default_topic__")
  {
    this->pub = this->node->Advertise<msgs::IMU>(
        this->sdf->GetElement("imu")->GetValueString("topic"));
  }
  else
  {
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/imu";
    boost::replace_all(topicName, "::", "/");

    this->pub = this->node->Advertise<msgs::IMU>(topicName);
  }
}

//////////////////////////////////////////////////
void ImuSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = boost::shared_dynamic_cast<physics::Link>(
      this->world->GetEntity(this->parentName));

  if (!this->parentEntity)
  {
    gzthrow("IMU has invalid paret[" + this->parentName +
            "]. Must be a link\n");
  }
  this->referencePose = this->pose + this->parentEntity->GetWorldPose();
  this->lastLinearVel = this->referencePose.rot.RotateVector(
    this->parentEntity->GetWorldLinearVel());

  this->gravity = this->world->GetPhysicsEngine()->GetGravity();
}

//////////////////////////////////////////////////
void ImuSensor::Init()
{
}

//////////////////////////////////////////////////
void ImuSensor::Fini()
{
}

//////////////////////////////////////////////////
math::Vector3 ImuSensor::GetAngularVelocity() const
{
  return msgs::Convert(this->imuMsg.angular_velocity());
}

//////////////////////////////////////////////////
math::Vector3 ImuSensor::GetLinearAcceleration() const
{
  return msgs::Convert(this->imuMsg.linear_acceleration());
}

//////////////////////////////////////////////////
math::Quaternion ImuSensor::GetOrientation() const
{
  return msgs::Convert(this->imuMsg.orientation());
}

//////////////////////////////////////////////////
void ImuSensor::SetReferencePose()
{
  this->referencePose = this->pose + this->parentEntity->GetWorldPose();
}

//////////////////////////////////////////////////
void ImuSensor::UpdateImpl(bool /*_force*/)
{
  double dt = (this->world->GetSimTime() - this->lastMeasurementTime).Double();

  this->lastMeasurementTime = this->world->GetSimTime();

  this->imuMsg.set_entity_name(this->parentName);

  // Set the time stamp
  msgs::Set(this->imuMsg.mutable_stamp(), this->world->GetSimTime());

  math::Pose parentEntityPose = this->parentEntity->GetWorldPose();
  math::Pose imuPose = this->pose + parentEntityPose;

  // Set the IMU orientation
  msgs::Set(this->imuMsg.mutable_orientation(),
            imuPose.rot * this->referencePose.rot.GetInverse());

  // Set the IMU angular velocity
  msgs::Set(this->imuMsg.mutable_angular_velocity(),
            imuPose.rot.GetInverse().RotateVector(
            this->parentEntity->GetWorldAngularVel()));

  // get linear velocity in world frame
  math::Vector3 imuWorldLinearVel =
    this->parentEntity->GetWorldLinearVel(this->pose.pos);

  // Compute and set the IMU linear acceleration
  if (dt > 0.0)
  {
    this->linearAcc = imuPose.rot.GetInverse().RotateVector(
      (imuWorldLinearVel - this->lastLinearVel) / dt);
    this->lastLinearVel = imuWorldLinearVel;
  }

  // Add contribution from gravity
  this->linearAcc += imuPose.rot.GetInverse().RotateVector(this->gravity);

  msgs::Set(this->imuMsg.mutable_linear_acceleration(), this->linearAcc);

  if (this->pub)
    this->pub->Publish(this->imuMsg);
}
