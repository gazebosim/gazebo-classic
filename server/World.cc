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
#include <fstream>

#include "Global.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "PhysicsEngine.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "UpdateParams.hh"
#include "World.hh"
#include "Geom.hh"

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  this->physicsEngine = new ODEPhysics();
  this->server=0;
  this->simIface=0;

  this->showBoundingBoxes = false;
  this->showJoints = false;
  this->wireframe = false;
  this->showPhysics = false;

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

  this->geometries.clear();

  GZ_DELETE (this->physicsEngine)
  GZ_DELETE (this->server)
  GZ_DELETE (this->simIface)

}


////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(XMLConfigNode *rootNode, int serverId)
{
  assert(serverId >= 0);

  // Create the server object (needs to be done before models initialize)
  this->server = new Server();

  try
  {
    this->server->Init(serverId, true );
  }
  catch ( std::string err)
  {
    gzthrow (err);
  }

   // Create the simulator interface
  this->simIface = new SimulationIface();
  this->simIface->Create(this->server, "default" );

  this->LoadEntities(rootNode, NULL);

  /*std::vector<Model*>::iterator miter;
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    this->SetModelPose(*miter, (*miter)->GetPose() + Global::poseOffset);
  }*/

  this->physicsEngine->Load(rootNode);
}

////////////////////////////////////////////////////////////////////////////////
// Save the world
void World::Save(XMLConfigNode *node)
{
  std::vector< Model* >::iterator miter;
  // Save all the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Save();
  }

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
  this->simIface->data->pause = Simulator::Instance()->IsPaused();
  this->simIface->Unlock();

  this->physicsEngine->Init();

  this->toAddModels.clear();
  this->toDeleteModels.clear();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
int World::Update()
{
  UpdateParams params;
  std::vector< Model* >::iterator miter;
  std::vector< Model* >::iterator miter2;

  params.stepTime = this->physicsEngine->GetStepTime();
  
  // Update all the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if (*miter)
    {
      (*miter)->Update(params);
    }
  }

  if (!Simulator::Instance()->IsPaused())
  {
    this->physicsEngine->Update();
  }

  this->UpdateSimulationIface();

  // Copy the newly created models into the main model vector
  std::copy(this->toAddModels.begin(), this->toAddModels.end(), 
      std::back_inserter(this->models));
  this->toAddModels.clear();


  // Remove and delete all models that are marked for deletion
  for (miter=this->toDeleteModels.begin(); 
       miter!=this->toDeleteModels.end(); miter++)
  {
    this->models.erase(
        std::remove(this->models.begin(), this->models.end(), *miter) );
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
// Get the simulation time, this method is replicated in Simulator class
double World::GetSimTime() const
{
  return Simulator::Instance()->GetSimTime();
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
  if (Simulator::Instance()->GetIterations() == 0)
    this->models.push_back(model);
  else
  {
    model->Init();
    this->toAddModels.push_back(model);
  }

  if (parent != NULL)
    model->Attach(node->GetChild("attach"));
    //model->Attach();

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

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to a model based on a name
Model *World::GetModelByName(std::string modelName)
{
  std::vector< Model *>::iterator iter;

  for (iter = models.begin(); iter != models.end(); iter++)
  {
    if ((*iter)->GetName() == modelName)
      return (*iter);
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
///  Get an iterator over the models
std::vector<Model*> &World::GetModels()
{
  return this->models;
}

///////////////////////////////////////////////////////////////////////////////
// Reset the simulation to the initial settings
void World::Reset()
{
  std::vector< Model* >::iterator miter;

  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    (*miter)->Reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
// The plane and heighweighmap will not be registered
void World::RegisterGeom(Geom *geom)
{
  this->geometries.push_back(geom);
}

////////////////////////////////////////////////////////////////////////////////
// True if the bounding boxes of the models are being shown 
bool World::GetShowBoundingBoxes()
{
  return this->showBoundingBoxes;
}

////////////////////////////////////////////////////////////////////////////////
// Set if the bounding boxes are shown or no
void World::SetShowBoundingBoxes(bool show)
{
  this->showBoundingBoxes = show;

  std::vector< Geom *>::iterator iter;

  for (iter = geometries.begin(); iter != geometries.end(); iter++)
  {
    (*iter)->ShowBoundingBox(this->showBoundingBoxes);
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Get wheter to show the joints
bool World::GetShowJoints()
{
  return this->showJoints;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the joints
void World::SetShowJoints(bool show)
{
  this->showJoints = show;

  std::vector< Geom *>::iterator iter;

  for (iter = geometries.begin(); iter != geometries.end(); iter++)
  {
    (*iter)->ShowJoints(this->showJoints);
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Set to view as wireframe
void World::SetWireframe( bool wire )
{
  this->wireframe = wire;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether to view as wireframe
bool World::GetWireframe()
{
  return this->wireframe;
}


////////////////////////////////////////////////////////////////////////////////
/// Get wheter to show the joints
bool World::GetShowPhysics()
{
  return this->showPhysics;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the joints
void World::SetShowPhysics(bool show)
{
  this->showPhysics = show;

  std::vector< Geom *>::iterator iter;

  for (iter = geometries.begin(); iter != geometries.end(); iter++)
  {
    (*iter)->ShowPhysics(this->showPhysics);
  }

}



////////////////////////////////////////////////////////////////////////////////
// Update the simulation interface
void World::UpdateSimulationIface()
{
  this->simIface->Lock(1);

  if (this->simIface->opened)
  {
    this->simIface->data->simTime = Simulator::Instance()->GetSimTime();
    this->simIface->data->pauseTime = Simulator::Instance()->GetPauseTime();
    this->simIface->data->realTime = Simulator::Instance()->GetRealTime();

    if (this->simIface->data->reset)
    {
      this->Reset();
      this->simIface->data->reset = 0;
    }

    // If the model_name is set, then a request has been received
    if (strcmp((char*)this->simIface->data->model_name,"")!=0)
    {
      /// Get the model requested
      Model *model = this->GetModelByName((char*)this->simIface->data->model_name);
      if (model)
      {
        std::string req = (char*)this->simIface->data->model_req;
        if (req == "get_pose")
        {
          Pose3d pose = model->GetPose();
          Vector3 rot = pose.rot.GetAsEuler();

          this->simIface->data->model_pose.pos.x = pose.pos.x;
          this->simIface->data->model_pose.pos.y = pose.pos.y;
          this->simIface->data->model_pose.pos.z = pose.pos.z;


          this->simIface->data->model_pose.roll = rot.x;
          this->simIface->data->model_pose.pitch = rot.y;
          this->simIface->data->model_pose.yaw = rot.z;
        }
        else if (req == "set_pose3d")
        {
          Pose3d pose;

          pose.pos.x = this->simIface->data->model_pose.pos.x;
          pose.pos.y = this->simIface->data->model_pose.pos.y;
          pose.pos.z = this->simIface->data->model_pose.pos.z;
          pose.rot.SetFromEuler(Vector3(this->simIface->data->model_pose.roll,
                this->simIface->data->model_pose.pitch,
                this->simIface->data->model_pose.yaw));
          model->SetPose(pose);
        }
        else if (req == "set_pose2d")
        {
          Pose3d pose = model->GetPose();
          Vector3 rot = pose.rot.GetAsEuler();

          pose.pos.x = this->simIface->data->model_pose.pos.x;
          pose.pos.y = this->simIface->data->model_pose.pos.y;

          pose.rot.SetFromEuler(Vector3(rot.x, rot.y,
                this->simIface->data->model_pose.yaw));
          model->SetPose(pose);
        }

      }

      strcpy((char*)this->simIface->data->model_name, "");
    }
  }
  this->simIface->Unlock();
}

