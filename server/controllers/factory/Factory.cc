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
 * Desc: Factory Position2d controller.
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
#include "Factory.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("factory", Factory);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Factory::Factory(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Factory controller requires a Model as its parent");

  // Prefix and suffix for xml files

  this->xmlPrefix = "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  this->xmlSuffix = "</gazebo:world>";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Factory::~Factory()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Factory::LoadChild(XMLConfigNode *node)
{
  this->factoryIface = dynamic_cast<FactoryIface*>(this->ifaces[0]);

  if (!this->factoryIface)
    gzthrow("Factory controller requires a factoryIface");
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Factory::InitChild()
{
  // initialize newModel to blank
  strcpy((char*)this->factoryIface->data->newModel,"");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Factory::UpdateChild()
{
  // If there is a string, then add the contents to the world
  this->factoryIface->Lock(1);
  if (strcmp((const char*)this->factoryIface->data->newModel,"")!=0)
  {
    //std::cout << " factory update: " << this->factoryIface->data->newModel << std::endl;
    std::string xmlString;
    std::string xmlMiddle = (const char*)(this->factoryIface->data->newModel);
    
    // Strip leading <?xml...?> tag, if present, to allow the client to
    // pass the contents of a valid .model file
    std::string xmlVersion = "<?xml version=\"1.0\"?>";
    int i = xmlMiddle.find(xmlVersion);
    if(i >= 0)
      xmlMiddle.replace(i, xmlVersion.length(), "");

    xmlString = this->xmlPrefix + xmlMiddle + this->xmlSuffix;

    // Add the new models into the World
    World::Instance()->InsertEntity( xmlString);

    strcpy((char*)this->factoryIface->data->newModel,"");
  }

  // Attempt to delete a model by name, if the string is present
  /*if (strcmp((const char*)this->factoryIface->data->deleteModel,"")!=0)
  {
    World::Instance()->DeleteEntity((const char*)this->factoryIface->data->deleteModel);

    strcpy((char*)this->factoryIface->data->deleteModel,"");
  }*/
  this->factoryIface->Unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Factory::FiniChild()
{
}

