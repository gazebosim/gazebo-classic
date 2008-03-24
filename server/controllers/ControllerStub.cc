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
 * Desc: Stubbed out controller
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "ControllerStub.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("controller_stub", ControllerStub);

////////////////////////////////////////////////////////////////////////////////
// Constructor
ControllerStub::ControllerStub(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("ControllerStub controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ControllerStub::~ControllerStub()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void ControllerStub::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<Your_Iface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("ControllerStub controller requires a <type> Iface");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void ControllerStub::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void ControllerStub::UpdateChild(UpdateParams &params)
{
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void ControllerStub::FiniChild()
{
}
