/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo/common/Exception.hh"

#include "gazebo/math/Vector3.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ImuSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor);

//////////////////////////////////////////////////
ImuSensor::ImuSensor()
    : Sensor()
{
}

//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::Load(sdf::ElementPtr _node)
{
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
Pose ImuSensor::GetVelocity()
{
  return this->imuVel;
}

//////////////////////////////////////////////////
void ImuSensor::Update()
{
//  if (this->active)
  {
    Vector3 velocity;
    Pose poseDelta;
    double heading;
    double v1;

    double vlong, vlat;

    // Quatern rot;
    Vector3 rot;
    Vector3 pose;

    // Get the pose of the sensor body (global cs)
    poseDelta = this->body->GetWorldPose() - this->prevPose;
    this->prevPose = this->body->GetWorldPose();

    velocity = this->body->GetWorldLinearVel();
    rot = this->body->GetWorldPose().rot.GetAsEuler();
    pose = this->body->GetWorldPose().pos;

    heading = atan2(velocity.y, velocity.x);

    v1 = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2));

    vlong = v1 * cos(heading - rot.z);
    vlat = v1 * sin(heading - rot.z);

    this->imuVel.pos.x = vlong;
    this->imuVel.pos.y = vlat;

    this->imuVel.pos.z = 0;

    /// \TODO storing x,y,z components in a quaternion seems like a bad idea
    velocity = this->body->GetWorldAngularVel();
    this->imuVel.rot.x = velocity.x;
    this->imuVel.rot.y = velocity.y;
    this->imuVel.rot.z = velocity.z;

    this->eulerAngles = rot;
  }
}
