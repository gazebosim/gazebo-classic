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
 * Desc: Factory Position2d controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "gazebo.h"
#include "common/GazeboError.hh"
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
  this->factoryIface = dynamic_cast<libgazebo::FactoryIface*>(this->GetIface("factory"));
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
    this->GetWorld()->InsertEntity( xmlString);

    strcpy((char*)this->factoryIface->data->newModel,"");
  }

  // Attempt to delete a model by name, if the string is present
  /*if (strcmp((const char*)this->factoryIface->data->deleteModel,"")!=0)
  {
    this->GetWorld()->DeleteEntity((const char*)this->factoryIface->data->deleteModel);

    strcpy((char*)this->factoryIface->data->deleteModel,"");
  }*/
  this->factoryIface->Unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Factory::FiniChild()
{
}

