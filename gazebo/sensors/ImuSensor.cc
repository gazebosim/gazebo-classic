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

#include <assert.h>
#include <float.h>
#include <sstream>

#include "SensorFactory.hh"
// #include "World.hh"
// #include "PhysicsEngine.hh"
#include "common/Exception.hh"
#include "ImuSensor.hh"

#include "math/Vector3.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor);

//////////////////////////////////////////////////
ImuSensor::ImuSensor(Body *body)
    : Sensor(body)
{
  this->active = false;

  this->typeName = "imu";
}


//////////////////////////////////////////////////
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////
void ImuSensor::LoadChild(XMLConfigNode *node)
{
  if (this->body == NULL)
  {
    gzthrow("Null body in the IMU sensor");
  }
}

//////////////////////////////////////////////////
void ImuSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
}

//////////////////////////////////////////////////
void ImuSensor::InitChild()
{
  Pose bodyPose;
  bodyPose = this->body->GetWorldPose();
  this->prevPose = bodyPose;
}

void ImuSensor::FiniChild()
{
}

Pose ImuSensor::GetVelocity()
{
  return this->imuVel;
}

Vector3 ImuSensor::GetEulerAngles()
{
  return this->eulerAngles;
}

//////////////////////////////////////////////////
void ImuSensor::UpdateChild()
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

    /// FIXME storing x,y,z components in a quaternion seems like a bad idea
    /// @todo storing x,y,z components in a quaternion seems like a bad idea
    velocity = this->body->GetWorldAngularVel();
    this->imuVel.rot.x = velocity.x;
    this->imuVel.rot.y = velocity.y;
    this->imuVel.rot.z = velocity.z;

    this->eulerAngles = rot;
  }
}


