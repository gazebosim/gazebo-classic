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
#include <sys/time.h> //gettimeofday

#include "GraphicsIfaceHandler.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "PhysicsEngine.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "World.hh"

#include "OpenAL.hh"

#include "Geom.hh"

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  this->server = NULL;
  this->simIface = NULL;
  this->showBoundingBoxes = false;
  this->showJoints = false;
  this->wireframe = false;
  this->showPhysics = false;
  this->physicsEngine = NULL;
  this->server = NULL;
  this->graphics = NULL;
  this->openAL = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  this->Close();
}

////////////////////////////////////////////////////////////////////////////////
// Closes the world, free resources and interfaces
void World::Close()
{
  std::vector< Model* >::iterator miter;
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    if (*miter)
    {
      delete (*miter);
      (*miter) = NULL;
    }
  }
  this->models.clear();
  this->geometries.clear();

  if (this->physicsEngine)
  {
    delete this->physicsEngine;
    this->physicsEngine = NULL;
  }

  if (this->server)
  {
    delete this->server;
    this->server =NULL;
  }

  try
  {
    if (this->simIface)
    {
      delete this->simIface;
      this->simIface = NULL;
    }
  }
  catch (std::string e)
  {
    gzthrow(e);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(XMLConfigNode *rootNode, unsigned int serverId)
{
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
  try
  {
    this->simIface = new SimulationIface();
    this->simIface->Create(this->server, "default" );
  }
  catch (std::string err)
  {
    gzthrow(err);
  }

  // Create the graphics iface handler
  this->graphics = new GraphicsIfaceHandler();
  this->graphics->Load("default");

  // Load OpenAL audio 
  if (rootNode->GetChild("openal","audio"))
  {
    //this->openAL = new OpenALAPI();
    //this->openAL->Load(rootNode->GetChild("openal", "audio"));
    this->openAL = OpenAL::Instance();
    this->openAL->Load(rootNode->GetChild("openal", "audio"));
  }

  this->physicsEngine = new ODEPhysics(); //TODO: use exceptions here

  this->LoadEntities(rootNode, NULL, false);

  /*std::vector<Model*>::iterator miter;
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    this->SetModelPose(*miter, (*miter)->GetPose() + Global::poseOffset);
  }*/

  this->physicsEngine->Load(rootNode);

}

////////////////////////////////////////////////////////////////////////////////
// Save the world
void World::Save(std::string &prefix, std::ostream &stream)
{
  std::vector< Model* >::iterator miter;

  // Save all the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if ( (*miter)->GetParent() == NULL)
    {
      (*miter)->Save(prefix, stream);
      stream << "\n";
    }
  }

}


////////////////////////////////////////////////////////////////////////////////
// Initialize the world
void World::Init()
{
  std::vector< Model* >::iterator miter;

  this->simPauseTime = 0;

  // Init all models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Init();
  }

  // Initialize the physics engine
  this->physicsEngine->Init();

  // Initialize openal
  if (this->openAL)
    this->openAL->Init();

  this->toAddModels.clear();
  this->toDeleteModels.clear();
  this->toLoadEntities.clear();

  this->graphics->Init();

}

////////////////////////////////////////////////////////////////////////////////
// Update the world
void World::Update()
{

  if (this->simPauseTime > 0)
  {
    if (Simulator::Instance()->GetSimTime() >= this->simPauseTime)
    {
      //printf("SimTime[%f] PauseTime[%f]\n", Simulator::Instance()->GetSimTime(), this->simPauseTime);

      this->simPauseTime = 0;
      Simulator::Instance()->SetPaused(true);

      // Tell the simiface that it's okay to trigger the go ack
      this->simIface->GoAckPost();
    }
    else
    {
      Simulator::Instance()->SetPaused(false);
    }
  }

#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif

  // Update all the models
  std::vector< Model* >::iterator miter;
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if (*miter)
    {
      (*miter)->Update();
    }
  }

#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << " World::Update() ALL Models update DT(" << tmpT2-tmpT1 << ")" << std::endl;
#endif

  if (!Simulator::Instance()->IsPaused() &&
       Simulator::Instance()->GetPhysicsEnabled())
  {
    this->physicsEngine->UpdatePhysics();
  }

  this->graphics->Update();

#ifdef TIMING
  double tmpT4 = Simulator::Instance()->GetWallTime();
  std::cout << " World::Update() Physics engine DT(" << tmpT4-tmpT2 << ")" << std::endl;
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Process messages
void World::ProcessMessages()
{
  this->UpdateSimulationIface();
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
{
  std::vector< Model* >::iterator miter;

  if (this->graphics)
    delete this->graphics;

  // Finalize the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Fini();
  }

  this->physicsEngine->Fini();

  // Done with the external interface
  try
  {
    this->simIface->Destroy();
  }
  catch (std::string e)
  { 
    gzmsg(-1) << "Problem destroying simIface[" << e << "]\n";
  }

  try
  {
    this->server->Fini();
  }
  catch (std::string e)
  {
    gzthrow(e);
  }

  // Close the openal server
  if (this->openAL)
    this->openAL->Fini();
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

///////////////////////////////////////////////////////////////////////////////
// Load a model
void World::LoadEntities(XMLConfigNode *node, Model *parent, bool removeDuplicate)
{
  XMLConfigNode *cnode;
  Model *model = NULL;

  if (node->GetNSPrefix() != "")
  {
    // Check for model nodes
    if (node->GetNSPrefix() == "model")
    {
      model = this->LoadModel(node, parent, removeDuplicate);
    }
  }

  // Load children
  for (cnode = node->GetChild(); cnode != NULL; cnode = cnode->GetNext())
  {
    this->LoadEntities( cnode, model, removeDuplicate );
  }

}

////////////////////////////////////////////////////////////////////////////////
// Add a new entity to the world
void World::InsertEntity( std::string xmlString)
{
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
  this->toLoadEntities.push_back( xmlString );
}

////////////////////////////////////////////////////////////////////////////////
// Load all the entities that have been queued
void World::ProcessEntitiesToLoad()
{
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
  std::vector< std::string >::iterator iter;

  for (iter = this->toLoadEntities.begin(); 
       iter != this->toLoadEntities.end(); iter++)
  {
    // Create the world file
    XMLConfig *xmlConfig = new XMLConfig();

    // Load the XML tree from the given string
    try
    {
      xmlConfig->LoadString( *iter );
    }
    catch (gazebo::GazeboError e)
    {
      gzerr(0) << "The world could not load the XML data [" << e << "]\n";
      continue;
    }

    this->LoadEntities( xmlConfig->GetRootNode(), NULL, true); 
    delete xmlConfig;
  }
 
  this->toLoadEntities.clear(); 
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
    {
      (*miter)->Fini();
      this->toDeleteModels.push_back(*miter);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Load a model
Model *World::LoadModel(XMLConfigNode *node, Model *parent, bool removeDuplicate)
{
  Pose3d pose;
  Model *model = new Model(parent);

  //model->SetParent(parent);
  // Load the model
  model->Load( node, removeDuplicate );

  // Set the model's pose (relative to parent)
  this->SetModelPose(model, model->GetInitPose());

  // Add the model to our list
  if (Simulator::Instance()->GetSimTime() == 0)
    this->models.push_back(model);
  else
  {
    model->Init();
    this->toAddModels.push_back(model);
  }

  if (parent != NULL)
    model->Attach(node->GetChild("attach"));

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
    this->SetModelPose((*miter), (*miter)->GetInitPose());
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
  SimulationRequestData *response = NULL;

  //TODO: Move this method to simulator? Hard because of the models
  this->simIface->Lock(1);

  /* This call releases our lock, which can lead to hard-to-track-down
   * synchronization bugs.  Besides, it's a small optimization at best
  if (this->simIface->GetOpenCount() <= 0)
  {
    this->simIface->Unlock();
    return;
  }
  */

  response = this->simIface->data->responses;

  this->simIface->data->simTime = Simulator::Instance()->GetSimTime();
  this->simIface->data->pauseTime = Simulator::Instance()->GetPauseTime();
  this->simIface->data->realTime = Simulator::Instance()->GetRealTime();
  this->simIface->data->state = !Simulator::Instance()->GetUserPause();

  unsigned int requestCount = this->simIface->data->requestCount;

  // Make sure the request count is valid
  if (this->simIface->data->requestCount > GAZEBO_SIMULATION_MAX_REQUESTS)
  {
    gzerr(0) << "Request count[" << this->simIface->data->requestCount << "] greater than max allowable[" << GAZEBO_SIMULATION_MAX_REQUESTS << "]\n";

    requestCount = GAZEBO_SIMULATION_MAX_REQUESTS;
  }

  // Process all the requests
  for (unsigned int i=0; i < requestCount; i++)
  {
    SimulationRequestData *req = &(this->simIface->data->requests[i]);

    switch (req->type)
    {
      case SimulationRequestData::PAUSE: 
        Simulator::Instance()->SetUserPause(
            !Simulator::Instance()->GetUserPause());
        break;

      case SimulationRequestData::RESET:
        this->Reset();
        break;

      case SimulationRequestData::SAVE:
        Simulator::Instance()->Save();
        break;

      case SimulationRequestData::SET_STATE:
        {
          Model *model = this->GetModelByName((char*)req->modelName);

          if (model)
          {
            Pose3d pose;
            Vector3 linearVel( req->modelLinearVel.x,
                               req->modelLinearVel.y,
                               req->modelLinearVel.z);
            Vector3 angularVel( req->modelAngularVel.x,
                                req->modelAngularVel.y,
                                req->modelAngularVel.z);
            Vector3 linearAccel( req->modelLinearAccel.x,
                                 req->modelLinearAccel.y,
                                 req->modelLinearAccel.z);
            Vector3 angularAccel( req->modelAngularAccel.x,
                                  req->modelAngularAccel.y,
                                  req->modelAngularAccel.z);


            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;
            pose.pos.z = req->modelPose.pos.z;

            // The the model's pose
            pose.rot.SetFromEuler(
                Vector3(
                  req->modelPose.roll, 
                  req->modelPose.pitch,
                  req->modelPose.yaw));
            model->SetPose(pose);

            linearVel = pose.rot.RotateVector(linearVel);
            angularVel = pose.rot.RotateVector(angularVel);

            linearAccel = pose.rot.RotateVector(linearAccel);
            angularAccel = pose.rot.RotateVector(angularAccel);

            // Set the model's linear and angular velocity
            model->SetLinearVel(linearVel);
            model->SetAngularVel(angularVel);

            // Set the model's linear and angular acceleration
            model->SetLinearAccel(linearAccel);
            model->SetAngularAccel(angularAccel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName 
                     << "] in simulation interface Set State Request.\n";
          }
          break;
        }
      case SimulationRequestData::SET_POSE3D:
        {
          Pose3d pose;
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;
            pose.pos.z = req->modelPose.pos.z;

            pose.rot.SetFromEuler(
                Vector3(req->modelPose.roll, 
                  req->modelPose.pitch,
                  req->modelPose.yaw));
            model->SetPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Set Pose 3d Request.\n";
          }

          break;
        }

      case SimulationRequestData::GET_POSE2D:
      case SimulationRequestData::GET_POSE3D:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Pose3d pose = model->GetPose();
            Vector3 rot = pose.rot.GetAsEuler();

            response->type = req->type;

            strcpy( response->modelName, req->modelName);
            response->modelPose.pos.x = pose.pos.x;
            response->modelPose.pos.y = pose.pos.y;
            response->modelPose.pos.z = pose.pos.z;

            response->modelPose.roll = rot.x;
            response->modelPose.pitch = rot.y;
            response->modelPose.yaw = rot.z;

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Pose 3d Request.\n";
          }

          break;
        }

      case SimulationRequestData::GO:
        {
          this->simPauseTime = Simulator::Instance()->GetSimTime() 
                                  + req->runTime * 10e-6;

          Simulator::Instance()->SetPaused(false);
          break;
        }

      case SimulationRequestData::SET_POSE2D:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Pose3d pose = model->GetPose();
            Vector3 rot = pose.rot.GetAsEuler();

            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;

            pose.rot.SetFromEuler(Vector3(rot.x, rot.y,
                  req->modelPose.yaw));
            model->SetPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Set Pose 2d Request.\n";
          }
          break;
        }

      default:
        gzerr(0) << "Unknown simulation iface request[" << req->type << "]\n";
        break;
    }

    this->simIface->data->requestCount = 0;
  }

  this->simIface->Unlock();


  std::vector< Model* >::iterator miter;

  // Copy the newly created models into the main model vector
  std::copy(this->toAddModels.begin(), this->toAddModels.end(),
      std::back_inserter(this->models));
  this->toAddModels.clear();


  // Remove and delete all models that are marked for deletion
  for (miter=this->toDeleteModels.begin();
      miter!=this->toDeleteModels.end(); miter++)
  {
//    (*miter)->Fini();
    this->models.erase(
        std::remove(this->models.begin(), this->models.end(), *miter) );
    delete *miter;
  }

  this->toDeleteModels.clear();
}

