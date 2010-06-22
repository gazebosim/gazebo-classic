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
/*
 * Desc: General steering controller for any number of wheels and configuration
 * Author: Jordi Polo
 * Date: 23 Dec 2007
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Steering_Position2d.hh"
#include "Wheel.hh"
#include "DriveWheel.hh"
#include "FullWheel.hh"
#include <string.h>


using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("steering_position2d", Steering_Position2d);

enum {DRIVE, STEER, FULL};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Steering_Position2d::Steering_Position2d(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Steering_Position2d controller requires a Model as its parent");

  this->enableMotors = true;
  this->cmdSteer=0;
  this->cmdSpeed=0;

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Steering_Position2d::~Steering_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Steering_Position2d::LoadChild(XMLConfigNode *node)
{

  XMLConfigNode *childNode;
  std::string jointName;
  std::string typestr;
  int type;
  Wheel *wheel;
  float torque, steerTorque;
  float maxAngle;
  float steerKp, steerKd;
  float defaultTorque, defaultSteerTorque, defaultKp, defaultKd;
  float defaultMaxAngle;


  this->myIface = dynamic_cast<libgazebo::PositionIface*>(this->GetIface("position"));

  //general
  defaultTorque = node->GetFloat("torque", 1000, 0);

  // Steering controller
  defaultSteerTorque = node->GetFloat("steerTorque", 100, 0);
  defaultKp = node->GetTupleDouble("steerPD", 0, 10.0);
  defaultKd = node->GetTupleDouble("steerPD", 1, 1.0);
  defaultMaxAngle = node->GetFloat("steerMaxAngle", DTOR(50), 0);


  childNode = node->GetChild("wheel");

  while (childNode)
  {
    jointName = childNode->GetString("joint", "", 1);
    typestr = childNode->GetString("type", "", 1);
    torque = childNode->GetFloat("torque", defaultTorque, 0);
    steerTorque = node->GetFloat("steerTorque", defaultSteerTorque, 0);
    steerKp = node->GetTupleDouble("steerPD", 0, defaultKp);
    steerKd = node->GetTupleDouble("steerPD", 1, defaultKd);
    maxAngle = node->GetFloat("steerMaxAngle", defaultMaxAngle, 0);

    wheel=new Wheel ();

    if (typestr=="drive")
    {
      DriveWheel *wheel=new DriveWheel();
      wheel->Connect(this->myParent->GetJoint(jointName), DRIVE);
      wheel->SetTorque(torque);
      wheels.push_back(wheel);
    }
    else
    {
      if (typestr=="steer")
        type=STEER;
      else
        type=FULL;

      FullWheel *wheel=new FullWheel();
      wheel->Connect(this->myParent->GetJoint(jointName), type);
      // If the wheel is not full, FMax2 should be 0 otherwise joint will lock
      if(type == FULL)
         wheel->SetTorque(torque);
      else
         wheel->SetTorque(0);
      wheel->SetSteerTorque(steerTorque);
      wheel->SetSteerPD(steerKp, steerKd);
      wheel->SetSteerMaxAngle(maxAngle);
      // wheel->SetSuspension(0.95, 0.9, 0.1); //TODO: we need step here
      wheels.push_back(wheel);
    }

    childNode= childNode->GetNext("wheel");
  }

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Steering_Position2d::InitChild()
{
  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  this->odomVel[0] = 0.0;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void Steering_Position2d::ResetChild()
{
  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  this->odomVel[0] = 0.0;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Steering_Position2d::UpdateChild()
{

  this->GetPositionCmd();

  //TODO: this limit can be defined in the joints! But it is more evident here?
  // Constrain the steering angle to lie within the stops

  std::vector<Wheel*>::iterator iter;

  for (iter=this->wheels.begin(); iter!=this->wheels.end(); iter++)
  {
    if (this->enableMotors)
      (*iter)->Update(this->cmdSpeed, this->cmdSteer);
    else
      (*iter)->Stop();
  }


//TODO: compute odometric info

  this->PutPositionData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Steering_Position2d::FiniChild()
{
}


//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Steering_Position2d::GetPositionCmd()
{

  if (this->myIface->Lock(1))
  {

    this->cmdSpeed = this->myIface->data->cmdVelocity.pos.x;
    this->cmdSteer = DTOR(this->myIface->data->cmdVelocity.yaw);

    this->enableMotors = this->myIface->data->cmdEnableMotors > 0;

    this->myIface->Unlock();
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Steering_Position2d::PutPositionData()
{
  if (this->myIface->Lock(1))
  {
    // TODO: Data timestamp
    this->myIface->data->head.time = Simulator::Instance()->GetSimTime().Double();

    this->myIface->data->pose.pos.x = this->odomPose[0];
    this->myIface->data->pose.pos.y = this->odomPose[1];
    this->myIface->data->pose.yaw = NORMALIZE(this->odomPose[2]);

    this->myIface->data->velocity.pos.x = this->odomVel[0];
    this->myIface->data->velocity.yaw = this->odomVel[2];

    // TODO
    this->myIface->data->stall = 0;

    this->myIface->Unlock();
  }
}
