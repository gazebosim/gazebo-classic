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
 */
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 * SVN: $Id$
 */

#include "Controller.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "Body.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "ControllerFactory.hh"
#include "Sensor.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor(Body *body)
  : Entity()
{
  this->body = body;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load(XMLConfigNode *node)
{
  this->LoadController( node->GetChildByNSPrefix("controller") );
  this->LoadChild(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update(UpdateParams &params)
{
  this->UpdateChild(params);
  this->controller->Update(params);
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the sensor
void Sensor::Fini()
{
  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Sensor::LoadController(XMLConfigNode *node)
{
  if (!node)
    gzthrow( "node parameter is NULL" );

  Iface *iface;
  XMLConfigNode *childNode;
  std::ostringstream stream;

  // Get the controller's type
  std::string controllerType = node->GetName();

  // Get the unique name of the controller
  std::string controllerName = node->GetString("name","",1);

  std::cout << "Sensor Controller Name[" << controllerName << "]\n";

  // Create the interface
  if ( (childNode = node->GetChildByNSPrefix("interface")) )
  {
    // Get the type of the interface (eg: laser)
    std::string ifaceType = childNode->GetName();

    std::cout << "Interface Type[" << ifaceType << "]\n";

    // Get the name of the iface 
    std::string ifaceName = childNode->GetString("name","",1);

    std::cout << "Iface Name[" << ifaceName << "]\n";

    // Use the factory to get a new iface based on the type
    iface = IfaceFactory::NewIface(ifaceType);

    // Create the iface
    iface->Create(World::Instance()->GetGzServer(), ifaceName);
  }
  else
  {
    stream << "No interface defined for " << controllerName << "controller";
    gzthrow(stream.str()); 
  }

  // Create the controller based on it's type
  this->controller = ControllerFactory::NewController(controllerType, iface, this);

  // Load the controller
  this->controller->Load(node);
}
