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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include "World.hh"
#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "ContactSensor.hh"
#include "World.hh"
#include "common/GazeboError.hh"
#include "ControllerFactory.hh"
#include "Simulator.hh"
#include "Generic_Bumper.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("bumper", Generic_Bumper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Generic_Bumper::Generic_Bumper(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<ContactSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Bumper controller requires a Contact Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Generic_Bumper::~Generic_Bumper()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Generic_Bumper::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<libgazebo::BumperIface*>(this->GetIface("bumper"));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Generic_Bumper::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Generic_Bumper::UpdateChild()
{
  this->myIface->Lock(1);

  this->myIface->data->bumper_count = this->myParent->GetGeomCount();

  this->myIface->data->head.time = this->myParent->GetWorld()->GetRealTime().Double();

  for (unsigned int i=0; i < this->myParent->GetGeomCount(); i++)
  {
    this->myIface->data->bumpers[i] = 
      this->myParent->GetGeomContactCount(i) > 0 ? 1 : 0; 
  }

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Generic_Bumper::FiniChild()
{
}
