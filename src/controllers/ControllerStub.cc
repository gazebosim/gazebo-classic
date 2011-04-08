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
 * Desc: Stubbed out controller
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "gazebo.h"
#include "common/Exception.hh"
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
