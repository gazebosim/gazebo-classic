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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

#include <assert.h>

#include "OgreAdaptor.hh"
#include "PhysicsEngine.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "ModelFactory.hh"
#include "gazebo.h"
#include "World.hh"

using namespace gazebo;

// static pointer to myself
World *World::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  //OgreAdaptor::Instance()->Init();

  this->physicsEngine = new ODEPhysics();

  this->server = NULL;
  this->simIface = NULL;

  this->pause = false;
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  delete this->physicsEngine;
}

////////////////////////////////////////////////////////////////////////////////
//Get an instance of this World
World *World::Instance()
{
  if (!myself)
  {
    myself = new World();
  }

  return myself;
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
int World::Load(XMLConfig *config, int serverId)
{
  assert(serverId >= 0);
  XMLConfigNode *rootNode = config->GetRootNode();

  if (OgreAdaptor::Instance()->Init(rootNode->GetChildByNSPrefix("rendering")) != 0)
  {
    std::cerr << "Failed to Initialize the OGRE Rendering system\n";
    return -1;
  }
  
  // Create the server object (needs to be done before models initialize)
  this->server = new Server();
  if (this->server->Init(serverId, true ) != 0)
    return -1;

   // Create the simulator interface
  this->simIface = new SimulationIface();
  if (this->simIface->Create(this->server, "default" ) != 0)
    return -1;

  this->LoadModel(rootNode, NULL);

  this->physicsEngine->Load();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the world
int World::Init()
{
  std::vector<Model*>::iterator iter;

  for (iter=this->models.begin(); iter!=this->models.end(); iter++)
  {
    (*iter)->Init();
  }

  // Set initial simulator state
  this->simIface->Lock(1);
  this->simIface->data->pause = this->pause;
  this->simIface->Unlock();

  this->physicsEngine->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
int World::Update()
{
  std::vector<Model*>::iterator iter;
  this->physicsEngine->Update();

  for (iter=this->models.begin(); iter!=this->models.end(); iter++)
  {
    (*iter)->Update();
  }

  OgreAdaptor::Instance()->Render();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
int World::Fini()
{
  std::vector<Model*>::iterator iter;

  // Finalize the models
  for (iter=this->models.begin(); iter!=this->models.end(); iter++)
  {
    (*iter)->Fini();
  }

  this->physicsEngine->Fini();

  // Done with the external interface
  this->simIface->Destroy();
  delete this->simIface;

  this->server->Fini();
  delete this->server;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Retun the libgazebo server
Server *World::GetGzServer() const
{
  return this->server;
}


////////////////////////////////////////////////////////////////////////////////
// Return the physics engine
PhysicsEngine *World::GetPhysicsEngine() const
{
  return this->physicsEngine;
}


////////////////////////////////////////////////////////////////////////////////
// Load a model
int World::LoadModel(XMLConfigNode *node, Model *parent)
{
  XMLConfigNode *cnode;
  Model *model = NULL;
  Pose3d pose;

  if (node->GetNSPrefix() != "")
  {
    // Check for model nodes
    if (node->GetNSPrefix() == "model")
    {

      if (node->GetName() == "xml")
      {
        model = new Model();
      }
      else
      {
        std::cout << "Model[" << node->GetName() << "]\n";

        // Instantiate the model
        model = ModelFactory::NewModel( node->GetName() );

        if (model == NULL)
        {
          std::cout << "unknown model class or class disabled [" << node->GetName() << "]\n";
          return 0;
        }
      }

      if (model)
      {
        model->SetType(node->GetName());

        // Recall the node this model is attached to, so we can save
        // back the data later.
        model->SetXMLConfigNode(node);

        // Set the id of the model
        model->SetName( node->GetString( "name", "", 0 ) );

        if (model->GetName() == "")
        {
          model->SetName( node->GetName() );
        }

        // Load the model
        if (model->Load( node ) != 0)
          return -1;

        // Get the position and orientation of the model (relative to parent)
        pose.Reset();
        pose.pos = node->GetVector3( "xyz", pose.pos );
        pose.rot = node->GetRotation( "rpy", pose.rot );

        // Set the model's pose (relative to parent)
        model->SetPose(pose);

        // Record the model's initial pose (for reseting)
        model->SetInitPose(pose);

        // Add the model to our list
        this->models.push_back(model);
      }
    }
  }

  // Load children
  for (cnode = node->GetChild(); cnode != NULL; cnode = cnode->GetNext())
  {
    if (this->LoadModel( cnode, model ) != 0)
      return -1;
  }

  return 0;
}

