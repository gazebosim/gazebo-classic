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
 * Desc: Position2d controller for a Pioneer2dx.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "HingeJoint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Pioneer2dx_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("pioneer2dx_position2d", Pioneer2dx_Position2d);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pioneer2dx_Position2d::Pioneer2dx_Position2d(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Pioneer2dx_Position2d controller requires a Model as its parent");

  this->enableMotors = true;

  this->wheelSpeed[RIGHT] = 0;
  this->wheelSpeed[LEFT] = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pioneer2dx_Position2d::~Pioneer2dx_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Pioneer2dx_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PositionIface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("Pioneer2dx_Position2d controller requires a PositionIface");

  this->wheelSep = 0.34;
  this->wheelDiam = 0.15;

  std::string leftJointName = node->GetString("leftJoint", "", 1);
  std::string rightJointName = node->GetString("rightJoint", "", 1);

  this->joints[LEFT] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(leftJointName));
  this->joints[RIGHT] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(rightJointName));

  if (!this->joints[LEFT])
    gzthrow("couldn't get left hinge joint");

  if (!this->joints[RIGHT])
    gzthrow("couldn't get right hinge joint");

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Pioneer2dx_Position2d::InitChild()
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
void Pioneer2dx_Position2d::UpdateChild(UpdateParams &params)
{
  bool opened = false;

  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;

  this->GetPositionCmd();

  wd = this->wheelDiam;
  ws = this->wheelSep;

  // Distance travelled by front wheels
  d1 = params.stepTime * wd / 2 * this->joints[LEFT]->GetAngleRate();
  d2 = params.stepTime * wd / 2 * this->joints[RIGHT]->GetAngleRate();

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / ws;

  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / params.stepTime;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / params.stepTime;

  if (this->enableMotors)
  {
    this->joints[LEFT]->SetParam( dParamVel, 
        this->wheelSpeed[LEFT] / this->wheelDiam * 2.0 );

    this->joints[RIGHT]->SetParam( dParamVel, 
        this->wheelSpeed[RIGHT] / this->wheelDiam * 2.0 );
    this->joints[LEFT]->SetParam( dParamFMax, 10.1 );
    this->joints[RIGHT]->SetParam( dParamFMax, 10.1 );

  }
  else
  {
    this->joints[LEFT]->SetParam( dParamVel, 0 ); 
    this->joints[RIGHT]->SetParam( dParamVel, 0 );

    this->joints[LEFT]->SetParam( dParamFMax, 0 );
    this->joints[RIGHT]->SetParam( dParamFMax, 0 );
  }

  this->PutPositionData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Pioneer2dx_Position2d::FiniChild()
{
}


//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Pioneer2dx_Position2d::GetPositionCmd()
{
  double vr, va;

  if (this->myIface->Lock(1))
  {

    vr = this->myIface->data->cmdVelocity.pos.x;
    va = this->myIface->data->cmdVelocity.yaw;

    this->enableMotors = this->myIface->data->cmdEnableMotors > 0;

    this->wheelSpeed[LEFT] = vr + va * this->wheelSep / 2;
    this->wheelSpeed[RIGHT] = vr - va * this->wheelSep / 2;

    this->myIface->Unlock();
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Pioneer2dx_Position2d::PutPositionData()
{
  if (this->myIface->Lock(1))
  {
    // TODO: Data timestamp
    this->myIface->data->time = World::Instance()->GetSimTime();

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
