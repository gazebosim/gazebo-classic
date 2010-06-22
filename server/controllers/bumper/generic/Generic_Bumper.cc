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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "ContactSensor.hh"
#include "World.hh"
#include "GazeboError.hh"
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

  this->myIface->data->head.time =Simulator::Instance()->GetRealTime().Double();

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
