/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: IMU sensor
 * Author: Matt Thompson
 * Date: 6 September 2008
 * SVN: $Id: $
*/

#include <assert.h>
#include <float.h>
#include <sstream>

#include "SensorFactory.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "GazeboError.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"
#include "Controller.hh"
#include "ImuSensor.hh"

#include "Vector3.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("imu", ImuSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
ImuSensor::ImuSensor(Body *body)
    : Sensor(body)
{
  this->active = false;

  this->typeName = "imu";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ImuSensor::~ImuSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an XMLConfig node
void ImuSensor::LoadChild(XMLConfigNode *node)
{
  if (this->body == NULL)
  {
    gzthrow("Null body in the IMU sensor");
  }

}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void ImuSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
}

//////////////////////////////////////////////////////////////////////////////
// Init the IMU
void ImuSensor::InitChild()
{
  Pose3d bodyPose;
  bodyPose = this->body->GetPose();
  this->prevPose = bodyPose;
}

void ImuSensor::FiniChild()
{
}

Pose3d ImuSensor::GetVelocity()
{
  return this->imuVel;
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void ImuSensor::UpdateChild()
{
//  if (this->active)
  {
    Vector3 velocity;
    Pose3d poseDelta;
    double heading;
    double v1;

    double vlong, vlat;

    //Quatern rot;
    Vector3 rot;
    Vector3 pose;

    // Get the pose of the sensor body (global cs)
    poseDelta = this->body->GetPose() - this->prevPose;
    this->prevPose = this->body->GetPose();

    velocity = this->body->GetLinearVel();
    rot = this->body->GetRotation().GetAsEuler();
    pose = this->body->GetPosition();

    heading = atan2(velocity.y, velocity.x);

    v1 = sqrt(pow(velocity.x,2) + pow(velocity.y,2));

    vlong = v1 * cos(heading - rot.z);
    vlat = v1 * sin(heading - rot.z);
   
    this->imuVel.pos.x = vlong; 
    this->imuVel.pos.y = vlat; 

    this->imuVel.pos.z = 0;
   
    velocity = this->body->GetAngularVel();
    this->imuVel.rot.x = velocity.x;
    this->imuVel.rot.y = velocity.y;
    this->imuVel.rot.z = velocity.z;

  }
}

}
