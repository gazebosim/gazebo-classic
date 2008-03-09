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
 * Desc: Controller base class.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "Model.hh"
#include "Sensor.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "Controller.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Controller::Controller(Entity *entity )
{
  if (!dynamic_cast<Model*>(entity) && !dynamic_cast<Sensor*>(entity))
  {
    gzthrow("The parent of a controller must be a Model or a Sensor");
  }

  this->parent = entity;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Controller::~Controller()
{
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Load the controller. Called once on startup
void Controller::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode;

  if (!this->parent)
    gzthrow("Parent entity has not been set");

  this->name = node->GetString("name","",1);

  this->updatePeriod = 1.0 / (node->GetDouble("updateRate", 10) + 1e-6);
  this->lastUpdate = -1e6;

  childNode = node->GetChildByNSPrefix("interface");

  // Create the interfaces
  while (childNode)
  {
    Iface *iface=0;
    
    // Get the type of the interface (eg: laser)
    std::string ifaceType = childNode->GetName();

    // Get the name of the iface 
    std::string ifaceName = childNode->GetString("name","",1);
    
    try 
    {
      // Use the factory to get a new iface based on the type
      iface = IfaceFactory::NewIface(ifaceType);
    }
    catch(...) //TODO: Show the exception text here (subclass exception?)
    {
      gzmsg(1) << "No manager for the interface " << ifaceType << " found. Disabled.\n";
      childNode = childNode->GetNextByNSPrefix("interface");
      continue;
    }
    
    // Create the iface
    iface->Create(World::Instance()->GetGzServer(), ifaceName);

    this->ifaces.push_back(iface);

    childNode = childNode->GetNextByNSPrefix("interface");
  }

  if (this->ifaces.size() <= 0)
  {
    std::ostringstream stream;
    stream << "No interface defined for " << this->name << " controller";
    gzthrow(stream.str()); 
  }

  this->LoadChild(node);
  this->xmlNode=node;
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller. 
void Controller::Save()
{
  //So far the controller can not change in any way, not rewrite
  this->SaveChild(this->xmlNode); 
}


////////////////////////////////////////////////////////////////////////////////
/// Initialize the controller. Called once on startup.
void Controller::Init()
{
  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void Controller::Reset()
{
  this->ResetChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the controller. Called every cycle.
void Controller::Update(UpdateParams &params)
{
  if (lastUpdate + updatePeriod <= Simulator::Instance()->GetSimTime()) {
    this->UpdateChild(params);
    lastUpdate = Simulator::Instance()->GetSimTime();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Finialize the controller. Called once on completion.
void Controller::Fini()
{
  std::vector<Iface*>::iterator iter;

  for (iter=this->ifaces.begin(); iter!=this->ifaces.end(); iter++)
  {
    delete *iter;
  }

  this->ifaces.clear();

  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
// Get the name of the controller
std::string Controller::GetName() const
{
  return this->name;
}
