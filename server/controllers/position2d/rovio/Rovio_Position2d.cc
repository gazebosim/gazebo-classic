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
 * Desc: Position2d controller for a Rovio drive.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: Rovio_Position2d.cc 8156 2009-07-30 02:33:46Z hsujohnhsu $
 */

#include "XMLConfig.hh"
#include "Model.hh"
#include "Global.hh"
#include "HingeJoint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "PhysicsEngine.hh"
#include "ControllerFactory.hh"
#include "Rovio_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("rovio_position2d", Rovio_Position2d);

enum {LEFT,RIGHT,BACK};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Rovio_Position2d::Rovio_Position2d(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Rovio_Position2d controller requires a Model as its parent");

  this->enableMotors = true;

  this->wheelSpeed[RIGHT] = 0;
  this->wheelSpeed[LEFT] = 0;

  this->prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&this->parameters);
  this->leftJointNameP = new ParamT<std::string>("leftJoint", "", 1);
  this->rightJointNameP = new ParamT<std::string>("rightJoint", "", 1);
  this->backJointNameP = new ParamT<std::string>("rightJoint", "", 1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Rovio_Position2d::~Rovio_Position2d()
{
  delete this->leftJointNameP;
  delete this->rightJointNameP;
  delete this->backJointNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Rovio_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PositionIface*>(this->GetIface("position"));

  this->leftJointNameP->Load(node);
  this->rightJointNameP->Load(node);

  this->joints[LEFT] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->leftJointNameP->GetValue()));
  this->joints[RIGHT] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->rightJointNameP->GetValue()));
  this->joints[BACK] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->leftJointNameP->GetValue()));

  if (!this->joints[LEFT])
    gzthrow("The controller couldn't get left hinge joint");

  if (!this->joints[RIGHT])
    gzthrow("The controller couldn't get right hinge joint");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Rovio_Position2d::InitChild()
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
// Load the controller
void Rovio_Position2d::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftJointNameP) << "\n";
  stream << prefix << *(this->rightJointNameP) << "\n";
  stream << prefix << *(this->torqueP) << "\n";
  stream << prefix << *(this->wheelDiamP) << "\n";
  stream << prefix << *(this->wheelSepP) << "\n";
}


////////////////////////////////////////////////////////////////////////////////
// Reset
void Rovio_Position2d::ResetChild()
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
void Rovio_Position2d::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  double stepTime;

  this->myIface->Lock(1);

  this->GetPositionCmd();

  wd = **(this->wheelDiamP);
  ws = **(this->wheelSepP);


  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - this->prevUpdateTime;
  this->prevUpdateTime = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  d1 = stepTime * wd / 2 * this->joints[LEFT]->GetAngleRate();
  d2 = stepTime * wd / 2 * this->joints[RIGHT]->GetAngleRate();

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / stepTime;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / stepTime;


  //if (this->enableMotors)
  {
    this->joints[LEFT]->SetParam( dParamVel,
                                  this->wheelSpeed[LEFT] /  (**(this->wheelDiamP) / 2.0) );

    this->joints[RIGHT]->SetParam( dParamVel,
                                   this->wheelSpeed[RIGHT] /  (**(this->wheelDiamP) / 2.0) );
    this->joints[LEFT]->SetParam( dParamFMax, **(this->torqueP) );
    this->joints[RIGHT]->SetParam( dParamFMax, **(this->torqueP) );

    //printf("Set Speed[%f %f]\n",this->wheelSpeed[LEFT], this->wheelSpeed[RIGHT]);
  }
  /*else
  {
    this->joints[LEFT]->SetParam( dParamVel, 0 );
    this->joints[RIGHT]->SetParam( dParamVel, 0 );

    this->joints[LEFT]->SetParam( dParamFMax, 0 );
    this->joints[RIGHT]->SetParam( dParamFMax, 0 );
  }*/

  this->PutPositionData();

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Rovio_Position2d::FiniChild()
{
}


//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Rovio_Position2d::GetPositionCmd()
{
  double vr, va;

  vr = this->myIface->data->cmdVelocity.pos.x;
  va = this->myIface->data->cmdVelocity.yaw;

  this->enableMotors = this->myIface->data->cmdEnableMotors > 0;

  this->wheelSpeed[LEFT] = vr + va * **(this->wheelSepP) / 2;
  this->wheelSpeed[RIGHT] = vr - va * **(this->wheelSepP) / 2;

}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Rovio_Position2d::PutPositionData()
{
  // TODO: Data timestamp
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime();

  this->myIface->data->pose.pos.x = this->odomPose[0];
  this->myIface->data->pose.pos.y = this->odomPose[1];
  this->myIface->data->pose.yaw = NORMALIZE(this->odomPose[2]);

  this->myIface->data->velocity.pos.x = this->odomVel[0];
  this->myIface->data->velocity.yaw = this->odomVel[2];

  // TODO
  this->myIface->data->stall = 0;
}
