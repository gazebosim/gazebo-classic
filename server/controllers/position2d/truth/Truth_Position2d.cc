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
 * Desc: Truth Position2d controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id:$
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "HingeJoint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Truth_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("truth_position2d", Truth_Position2d);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Truth_Position2d::Truth_Position2d(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Truth_Position2d controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Truth_Position2d::~Truth_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Truth_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PositionIface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("Truth_Position2d controller requires a PositionIface");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Truth_Position2d::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Truth_Position2d::UpdateChild(UpdateParams &params)
{
  Pose3d pose = this->myParent->GetPose();
  Vector3 rot = pose.rot.GetAsEuler();

  this->myIface->Lock(1);
  
  // TODO: Data timestamp
  this->myIface->data->time = World::Instance()->GetSimTime();

  this->myIface->data->pose.x = pose.pos.x;
  this->myIface->data->pose.y = pose.pos.y;
  this->myIface->data->pose.yaw = rot.z;

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Truth_Position2d::FiniChild()
{
}

