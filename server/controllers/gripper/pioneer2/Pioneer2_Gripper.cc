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
#include "SliderJoint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Pioneer2_Gripper.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("pioneer2_gripper", Pioneer2_Gripper);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pioneer2_Gripper::Pioneer2_Gripper(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Pioneer2_Gripper controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pioneer2_Gripper::~Pioneer2_Gripper()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Pioneer2_Gripper::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<GripperIface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("Pioneer2_Gripper controller requires a GripperIface");

  Param::Begin(&this->parameters);
  this->leftJointNameP = new ParamT<std::string>("leftJoint", "", 1);
  this->rightJointNameP = new ParamT<std::string>("rightJoint", "", 1);
  Param::End();

  this->joints[LEFT] = dynamic_cast<SliderJoint*>(this->myParent->GetJoint(this->leftJointNameP->GetValue()));
  this->joints[RIGHT] = dynamic_cast<SliderJoint*>(this->myParent->GetJoint(this->rightJointNameP->GetValue()));

  if (!this->joints[LEFT])
    gzthrow("couldn't get left slider joint");

  if (!this->joints[RIGHT])
    gzthrow("couldn't get right slider joint");

}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller
void Pioneer2_Gripper::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftJointNameP) << "\n";
  stream << prefix << *(this->rightJointNameP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Pioneer2_Gripper::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Pioneer2_Gripper::UpdateChild()
{
  /*double leftPaddleHiStop = this->joints[LEFT]->GetParam(dParamHiStop);
  double leftPaddleLoStop = this->joints[LEFT]->GetParam(dParamLoStop);
  double rightPaddleHiStop = this->joints[RIGHT]->GetParam(dParamHiStop);
  double rightPaddleLoStop = this->joints[RIGHT]->GetParam(dParamLoStop);

  double leftPaddlePos = this->joints[LEFT]->GetPosition();
  double rightPaddlePos = this->joints[RIGHT]->GetPosition();
  */

  this->myIface->Lock(1);

  switch( this->myIface->data->cmd)
  {
    case GAZEBO_GRIPPER_CMD_OPEN:
        this->joints[RIGHT]->SetParam(dParamVel,0.1);
        this->joints[LEFT]->SetParam(dParamVel, -0.1);
      break;

    case GAZEBO_GRIPPER_CMD_CLOSE:
      this->joints[RIGHT]->SetParam(dParamVel,-0.1);
      this->joints[LEFT]->SetParam(dParamVel,0.1);
      break;
  }

  this->joints[LEFT]->SetParam(dParamFMax,.01);
  this->joints[RIGHT]->SetParam(dParamFMax,.01);

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Pioneer2_Gripper::FiniChild()
{
}
