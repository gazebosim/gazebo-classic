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
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Pioneer2dx_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("pioneer2dx_position2d", Pioneer2dx_Position2d);

enum {LEFT, RIGHT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pioneer2dx_Position2d::Pioneer2dx_Position2d()
{
  this->joints[0] = NULL;
  this->joints[1] = NULL;
  this->enableMotors = true;

  this->wheelSpeed[0] = 0;
  this->wheelSpeed[1] = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pioneer2dx_Position2d::~Pioneer2dx_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
int Pioneer2dx_Position2d::LoadChild(XMLConfigNode *node)
{
  this->wheelSep = 0.35;
  this->wheelDiam = 0.19;

  std::string leftJointName = node->GetString("leftJoint", "", 1);
  std::string rightJointName = node->GetString("rightJoint", "", 1);

  this->joints[LEFT] = dynamic_cast<HingeJoint*>(this->model->GetJoint(leftJointName));
  this->joints[RIGHT] = dynamic_cast<HingeJoint*>(this->model->GetJoint(rightJointName));

  if (!this->joints[LEFT])
    throw GazeboError("Pioneer2dx_Position2d::LoadChild","couldn't get left hinge joint");

  if (!this->joints[RIGHT])
    throw GazeboError("Pioneer2dx_Position2d::LoadChild","couldn't get right hinge joint");

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
int Pioneer2dx_Position2d::InitChild()
{
  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  this->odomVel[0] = 0.0;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = 0.0;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
int Pioneer2dx_Position2d::UpdateChild(UpdateParams &params)
{
  // TODO: Step should be in a parameter of this function
  double step;
  double wd, ws;
  double d1, d2;
  double dr, da;

  wd = this->wheelDiam;
  ws = this->wheelSep;

  // Distance travelled by front wheels
  d1 = step * wd / 2 * this->joints[LEFT]->GetAngleRate();
  d2 = step * wd / 2 * this->joints[RIGHT]->GetAngleRate();

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / ws;
  
  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / step;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / step;

  this->GetPositionCmd();

  if (this->enableMotors)
  {
    this->joints[LEFT]->SetParam( dParamVel, 
        this->wheelSpeed[LEFT] / this->wheelDiam * 2 );

    this->joints[RIGHT]->SetParam( dParamVel, 
        this->wheelSpeed[RIGHT] / this->wheelDiam * 2 );
    
    this->joints[LEFT]->SetParam( dParamFMax, 1.1 );
    this->joints[RIGHT]->SetParam( dParamFMax, 1.1 );
  }
  else
  {
    this->joints[LEFT]->SetParam( dParamVel, 0.0 ); 
    this->joints[RIGHT]->SetParam( dParamVel, 0.0 );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
int Pioneer2dx_Position2d::FiniChild()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// The interface for the controller
void Pioneer2dx_Position2d::SetIface(Iface *iface)
{
  this->iface = dynamic_cast<PositionIface*>(iface);

  if (!this->iface)
    throw GazeboError("Pioneer2dx_Position2d::SetIface","iface is not of type PositionIface");
}

//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Pioneer2dx_Position2d::GetPositionCmd()
{
  double vr, va;

  vr = this->iface->data->cmdVelocity.x;
  va = this->iface->data->cmdVelocity.yaw;

  this->enableMotors = this->iface->data->cmdEnableMotors > 0;

  this->wheelSpeed[LEFT] = vr - va * this->wheelSep / 2;
  this->wheelSpeed[RIGHT] = vr + va * this->wheelSep / 2;
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Pioneer2dx_Position2d::PutPositionData()
{
  this->iface->Lock(1);
  
  // TODO: Data timestamp
  //this->iface->data->time = World::Instance()->GetSimTime();

  this->iface->data->pose.x = this->odomPose[0];
  this->iface->data->pose.y = this->odomPose[1];
  this->iface->data->pose.yaw = NORMALIZE(this->odomPose[2]);

  this->iface->data->velocity.x = this->odomVel[0];
  this->iface->data->velocity.yaw = this->odomVel[2];

  // TODO
  this->iface->data->stall = 0;

  this->iface->Unlock();
}
