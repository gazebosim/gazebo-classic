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
#include <sstream>
#include <sys/time.h>

#include "OgreDynamicLines.hh"
#include "Global.hh"
#include "OgreSimpleShape.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "OgreAdaptor.hh"
#include "PhysicsEngine.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "gazebo.h"
#include "UpdateParams.hh"
#include "World.hh"

using namespace gazebo;

// static pointer to myself
World *World::myself = NULL;

////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  this->physicsEngine = new ODEPhysics();

  this->pause = false;

  this->simTime = 0.0;
  this->pauseTime = 0.0;
  this->startTime = 0.0;

}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  std::vector< Model* >::iterator miter;

  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    delete (*miter);
  }
  this->models.clear();
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
void World::Load(XMLConfig *config, int serverId)
{
  assert(serverId >= 0);
  XMLConfigNode *rootNode(config->GetRootNode());

  try
  {
    OgreAdaptor::Instance()->Init(rootNode->GetChildByNSPrefix("rendering"));
  }
  catch (GazeboError e)
  {
    std::ostringstream stream;
    stream << "Failed to Initialize the OGRE Rendering system\n" 
              << e << "\n";
    gzthrow(stream.str());
  }

  // Create some basic shapes
  OgreSimpleShape::CreateSphere("unit_sphere",1.0, 32, 32);
  OgreSimpleShape::CreateBox("unit_box", Vector3(1,1,1));
  OgreSimpleShape::CreateCylinder("unit_cylinder", 1.0, 1.0, 1, 32);

  // Create the server object (needs to be done before models initialize)
  this->server = new Server();
  this->server->Init(serverId, true );

   // Create the simulator interface
  this->simIface = new SimulationIface();
  this->simIface->Create(this->server, "default" );

  this->LoadEntities(rootNode, NULL);

  this->physicsEngine->Load(rootNode);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the world
int World::Init()
{
  std::vector< Model* >::iterator miter;

  // Init all models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Init();
  }

  // Set initial simulator state
  this->simIface->Lock(1);
  this->simIface->data->pause = this->pause;
  this->simIface->Unlock();

  this->physicsEngine->Init();

  this->startTime = this->GetWallTime();

  this->tmpModels.clear();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
int World::Update()
{
  UpdateParams params;
  std::vector< Model* >::iterator miter;
  std::vector< Model* >::iterator miter2;

  this->simTime += this->physicsEngine->GetStepTime();
  params.stepTime = this->physicsEngine->GetStepTime();

  // Update all the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if (*miter)
      (*miter)->Update(params);
  }

  // Update the physics engine
  if (!Global::userPause && !Global::userStep ||
      (Global::userStep && Global::userStepInc))
  {
    this->physicsEngine->Update();
    Global::iterations++;

    Global::userStepInc = !Global::userStepInc;
  }
  else
  {
    this->pauseTime += this->physicsEngine->GetStepTime();
  }


  // Update the rendering engine
  //if (this->models.size() > 0)
  {
    OgreAdaptor::Instance()->Render();
  }

  this->simIface->Lock(1);
  this->simIface->data->simTime = this->GetSimTime();
  this->simIface->data->pauseTime = this->GetPauseTime();
  this->simIface->data->realTime = this->GetRealTime();
  this->simIface->Unlock();

  //std::copy(this->models.begin(), this->tmpModels.begin(), this->tmpModels.end());
  for (miter=this->tmpModels.begin(); miter!=this->tmpModels.end(); miter++)
  {
    this->models.push_back((*miter));
  }
  this->tmpModels.clear();

  for (miter=this->toDeleteModels.begin(); 
       miter!=this->toDeleteModels.end(); miter++)
  {
    //std::remove(this->models.begin(), this->models.end(), *miter);

    bool found = false;

    for (miter2=this->models.begin();
         miter2!=this->models.end();
         miter2++)
    {
      if (*miter == *miter2)
      {
        found = true;
        break;
      }
    }

    if (found)
    {
      this->models.erase(miter2);
    }
   
    delete *miter;
  }
  this->toDeleteModels.clear();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
int World::Fini()
{
  std::vector< Model* >::iterator miter;

  // Finalize the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Fini();
  }

  this->physicsEngine->Fini();

  // Done with the external interface
  this->simIface->Destroy();

  this->server->Fini();

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
// Get the simulation time
double World::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
double World::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
double World::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
double World::GetRealTime() const
{
  return this->GetWallTime() - this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the wall clock time
double World::GetWallTime() const
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec * 1e-6;
}

///////////////////////////////////////////////////////////////////////////////
// Load a model
int World::LoadEntities(XMLConfigNode *node, Model *parent)
{
  XMLConfigNode *cnode;
  Model *model = NULL;

  if (node->GetNSPrefix() != "")
  {
    // Check for model nodes
    if (node->GetNSPrefix() == "model")
    {
      model = this->LoadModel(node, parent);
    }
  }

  // Load children
  for (cnode = node->GetChild(); cnode != NULL; cnode = cnode->GetNext())
  {
    if (this->LoadEntities( cnode, model ) != 0)
      return -1;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Delete an entity by name
void World::DeleteEntity(const char *name)
{
  std::vector< Model* >::iterator miter;

  // Update all the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if ((*miter)->GetName() == name)
      this->toDeleteModels.push_back(*miter);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Load a model
Model *World::LoadModel(XMLConfigNode *node, Model *parent)
{
  Pose3d pose;
  Model *model = new Model();

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


  if (parent)
  {
    /*std::string bodyName = node->GetString("parentBody", "canonical");
      body = parent->GetBody(bodyName);

      if (!body)
      {
      std::ostringstream stream;
      stream << "body[" << bodyName << "] is not defined for parent model";
      gzthrow(stream);
      }
      */

    model->SetParent(parent);
  }

  // Load the model
  if (model->Load( node ) != 0)
    return NULL;

  // Get the position and orientation of the model (relative to parent)
  pose.Reset();
  pose.pos = node->GetVector3( "xyz", pose.pos );
  pose.rot = node->GetRotation( "rpy", pose.rot );

  // Set the model's pose (relative to parent)
  this->SetModelPose(model, pose);

  // Record the model's initial pose (for reseting)
  model->SetInitPose(pose);

  // Add the model to our list
  if (Global::iterations == 0)
    this->models.push_back(model);
  else
  {
    model->Init();
    this->tmpModels.push_back(model);
  }

  if (parent != NULL)
    model->Attach();

  return model;
}


////////////////////////////////////////////////////////////////////////////////
// Set the model pose and the pose of it's attached children 
void World::SetModelPose(Model *model , Pose3d pose)
{
  std::vector<Entity*>::iterator iter;
  Pose3d origPose, newPose, childPose;
  Model *parent = dynamic_cast<Model*>(model->GetParent());
  Model *child = NULL;

  // Get current pose
  origPose = model->GetPose();

  // Compute new global pose of the model
  if (parent)
    newPose = pose + parent->GetPose();
  else
    newPose = pose;

  // Recursively move children
  for (iter=model->GetChildren().begin(); 
       iter!=model->GetChildren().end(); iter++)
  {
    child = dynamic_cast<Model*>(*iter);

    if (child && child->GetParent() == model)
    {
      // Compute the current relative pose of the child
      childPose = child->GetPose() - origPose;

      // Compute the new global pose of the child
      childPose = childPose + newPose;

      // Compute the new child pose relative to the current model's pose
      childPose = childPose - origPose;

      this->SetModelPose( child, childPose );
    }
  }

  model->SetPose(newPose);
}
