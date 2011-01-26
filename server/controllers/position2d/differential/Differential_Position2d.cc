/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/*
 * Desc: Position2d controller for a Differential drive.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "XMLConfig.hh"
#include "Model.hh"
#include "Global.hh"
#include "Joint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "PhysicsEngine.hh"
#include "ControllerFactory.hh"
#include "Differential_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("differential_position2d", Differential_Position2d);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Differential_Position2d::Differential_Position2d(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  this->enableMotors = true;

  this->wheelSpeed[RIGHT] = 0;
  this->wheelSpeed[LEFT] = 0;

  this->prevUpdateTime = this->myParent->GetWorld()->GetSimTime();

  Param::Begin(&this->parameters);
  this->leftJointNameP = new ParamT<std::string>("leftJoint", "", 1);
  this->rightJointNameP = new ParamT<std::string>("rightJoint", "", 1);
  this->wheelSepP = new ParamT<float>("wheelSeparation", 0.34,1);
  this->wheelDiamP = new ParamT<float>("wheelDiameter", 0.15,1);
  this->torqueP = new ParamT<float>("torque", 10.0, 1);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Differential_Position2d::~Differential_Position2d()
{
  delete this->leftJointNameP;
  delete this->rightJointNameP;
  delete this->wheelSepP;
  delete this->wheelDiamP;
  delete this->torqueP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Differential_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<libgazebo::PositionIface*>(this->GetIface("position"));

  // the defaults are from pioneer2dx
  this->wheelSepP->Load(node);
  this->wheelDiamP->Load(node);
  this->torqueP->Load(node);

  this->leftJointNameP->Load(node);
  this->rightJointNameP->Load(node);

  this->joints[LEFT] = this->myParent->GetJoint(**this->leftJointNameP);
  this->joints[RIGHT] = this->myParent->GetJoint(**this->rightJointNameP);

  if (!this->joints[LEFT])
    gzthrow("The controller couldn't get left hinge joint");

  if (!this->joints[RIGHT])
    gzthrow("The controller couldn't get right hinge joint");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Differential_Position2d::InitChild()
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
void Differential_Position2d::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftJointNameP) << "\n";
  stream << prefix << *(this->rightJointNameP) << "\n";
  stream << prefix << *(this->torqueP) << "\n";
  stream << prefix << *(this->wheelDiamP) << "\n";
  stream << prefix << *(this->wheelSepP) << "\n";
}


////////////////////////////////////////////////////////////////////////////////
// Reset
void Differential_Position2d::ResetChild()
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
void Differential_Position2d::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time stepTime;

  this->myIface->Lock(1);

  this->GetPositionCmd();

  wd = **(this->wheelDiamP);
  ws = **(this->wheelSepP);


  stepTime = this->myParent->GetWorld()->GetSimTime() - this->prevUpdateTime;
  this->prevUpdateTime = this->myParent->GetWorld()->GetSimTime();

  // Distance travelled by front wheels
  d1 = stepTime.Double() * wd / 2 * this->joints[LEFT]->GetVelocity(0);
  d2 = stepTime.Double() * wd / 2 * this->joints[RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / stepTime.Double();
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / stepTime.Double();

  if (this->enableMotors)
  {
    this->joints[LEFT]->SetVelocity( 0, this->wheelSpeed[LEFT] /  
                                        (**(this->wheelDiamP) / 2.0) );

    this->joints[RIGHT]->SetVelocity( 0, this->wheelSpeed[RIGHT] /  
                                         (**(this->wheelDiamP) / 2.0) );

    this->joints[LEFT]->SetMaxForce( 0, **(this->torqueP) );
    this->joints[RIGHT]->SetMaxForce( 0, **(this->torqueP) );
  }

  this->PutPositionData();

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Differential_Position2d::FiniChild()
{
}


//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Differential_Position2d::GetPositionCmd()
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
void Differential_Position2d::PutPositionData()
{
  // TODO: Data timestamp
  this->myIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  this->myIface->data->pose.pos.x = this->odomPose[0];
  this->myIface->data->pose.pos.y = this->odomPose[1];
  this->myIface->data->pose.yaw = NORMALIZE(this->odomPose[2]);

  this->myIface->data->velocity.pos.x = this->odomVel[0];
  this->myIface->data->velocity.yaw = this->odomVel[2];

  // TODO
  this->myIface->data->stall = 0;
}
