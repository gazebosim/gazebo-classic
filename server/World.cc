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

#include "Body.hh"
#include "Factory.hh"
#include "GraphicsIfaceHandler.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "PhysicsEngine.hh"
#include "PhysicsFactory.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "SensorManager.hh"
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
  this->showContacts = false;
  this->showLights = false;
  this->showCameras = false;
  this->wireframe = false;
  this->showPhysics = false;
  this->perPixelLighting = true;
  this->physicsEngine = NULL;
  this->server = NULL;
  this->graphics = NULL;
  this->openAL = NULL;

  PhysicsFactory::RegisterAll();
  this->factory = NULL;

  Param::Begin(&this->parameters);
  this->threadsP = new ParamT<int>("threads",2,0);
  this->saveStateTimeoutP = new ParamT<Time>("saveStateResolution",0.1,0);
  this->saveStateBufferSizeP = new ParamT<unsigned int>("saveStateBufferSize",1000,0);
  Param::End();

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

  if (this->factory)
  {
    delete this->factory;
    this->factory = NULL;
  }

  if (this->saveStateTimeoutP)
    delete this->saveStateTimeoutP;
  this->saveStateTimeoutP = NULL;

  if (this->saveStateBufferSizeP)
    delete this->saveStateBufferSizeP;
  this->saveStateBufferSizeP = NULL;

  if (this->threadsP)
    delete this->threadsP;
  this->threadsP = NULL;

#ifdef USE_THREADPOOL
  if (this->threadPool)
    delete this->threadPool;
  this->threadPool = NULL;
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(XMLConfigNode *rootNode, unsigned int serverId)
{
  Simulator::Instance()->ConnectPauseSignal( 
      boost::bind(&World::PauseSlot, this, _1) );
  
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

  // Create the default factory
  this->factory = new Factory();

  // Create the graphics iface handler
  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->graphics = new GraphicsIfaceHandler();
    this->graphics->Load("default");
  }

  // Load OpenAL audio 
  if (rootNode->GetChild("openal","audio"))
  {
    this->openAL = OpenAL::Instance();
    this->openAL->Load(rootNode->GetChild("openal", "audio"));
  }

  XMLConfigNode *physicsNode = rootNode->GetChildByNSPrefix("physics");
  if (Simulator::Instance()->GetPhysicsEnabled() && physicsNode)
  {
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( physicsNode->GetName());
    if (this->physicsEngine == NULL)
      gzthrow("Unable to create physics engine\n");
  }

  this->LoadEntities(rootNode, NULL, false);

  /*std::vector<Model*>::iterator miter;
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    this->SetModelPose(*miter, (*miter)->GetPose() + Global::poseOffset);
  }*/

  this->physicsEngine->Load(rootNode);

  this->threadsP->Load(rootNode);
  this->saveStateTimeoutP->Load(rootNode);
  this->saveStateBufferSizeP->Load(rootNode);

  this->worldStates.resize(**this->saveStateBufferSizeP);
  this->worldStatesInsertIter = this->worldStates.begin();
  this->worldStatesEndIter = this->worldStates.begin();
  this->worldStatesCurrentIter = this->worldStatesInsertIter;


#ifdef USE_THREADPOOL
  // start a thread pool with X threads
  this->threadPool = new boost::threadpool::pool(this->threadsP->GetValue());
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Save the world
void World::Save(std::string &prefix, std::ostream &stream)
{
  std::vector< Model* >::iterator miter;

  std::cout << prefix << "  " << *(this->threadsP);
  std::cout << prefix << "  " << *(this->saveStateTimeoutP);
  std::cout << prefix << "  " << *(this->saveStateBufferSizeP);

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

  this->toDeleteModels.clear();
  this->toLoadEntities.clear();

  if (Simulator::Instance()->GetRenderEngineEnabled())
    this->graphics->Init();

  this->factory->Init();
  this->saveStateTimer.Start();
}

////////////////////////////////////////////////////////////////////////////////
// Primarily used to update the graphics interfaces
void World::GraphicsUpdate()
{
  this->graphics->Update();

  // Update all the models
  std::vector< Model* >::iterator miter;
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    if (*miter)
    {
      (*miter)->GraphicsUpdate();
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Update the world
void World::Update()
{
  if (this->simPauseTime > 0)
  {
    if (Simulator::Instance()->GetSimTime() >= this->simPauseTime)
    {
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

  {
    //DiagnosticTimer timer("World::Update Models");

    // Update all the models
    std::vector< Model* >::iterator miter;
    for (miter=this->models.begin(); miter!=this->models.end(); miter++)
    {
#ifdef USE_THREADPOOL
      this->threadPool->schedule(boost::bind(&Model::Update,(*miter)));
#else
      (*miter)->Update();
#endif
    }

#ifdef USE_THREADPOOL
  this->threadPool->wait();
#endif

  }

  /// Update all the sensors
  SensorManager::Instance()->Update();

  if (!Simulator::Instance()->IsPaused() &&
       Simulator::Instance()->GetPhysicsEnabled())
  {
    {
      //DiagnosticTimer timer("World::Update Physics");
      this->physicsEngine->UpdatePhysics();
    }

    /*if (this->saveStateTimer.GetElapsed() > **this->saveStateTimeoutP)
    {
      this->SaveState();
      this->saveStateTimer.Start();
    }*/
  }

  this->factory->Update();
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
{
  std::vector< Model* >::iterator miter;

  if (Simulator::Instance()->GetRenderEngineEnabled() && this->graphics)
    delete this->graphics;

  // Finalize the models
  for (miter=this->models.begin(); miter!=this->models.end(); miter++)
  {
    (*miter)->Fini();
  }

  if (this->physicsEngine)
    this->physicsEngine->Fini();

  // Done with the external interface
  try
  {
    if (this->simIface)
      this->simIface->Destroy();
  }
  catch (std::string e)
  { 
    gzmsg(-1) << "Problem destroying simIface[" << e << "]\n";
  }
  try
  {
    if (this->factory)
    {
      delete this->factory;
      this->factory = NULL;
    }
  }
  catch (std::string e)
  { 
    gzmsg(-1) << "Problem destroying factory[" << e << "]\n";
  }

  try
  {
    if (this->server)
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
      this->addEntitySignal(model);
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
  this->toLoadEntities.push_back( xmlString );
}

////////////////////////////////////////////////////////////////////////////////
// Load all the entities that have been queued
void World::ProcessEntitiesToLoad()
{

  if (!this->toLoadEntities.empty())
  {
    // maybe try try_lock here instead
    boost::recursive_mutex::scoped_lock lock(
        *Simulator::Instance()->GetMRMutex());

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
}

////////////////////////////////////////////////////////////////////////////////
// Load all the entities that have been queued
void World::ProcessEntitiesToDelete()
{
  if (!this->toDeleteModels.empty())
  {
    // maybe try try_lock here instead
    boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMDMutex());

    // Remove and delete all models that are marked for deletion
    std::vector< Model* >::iterator miter;
    for (miter=this->toDeleteModels.begin();
        miter!=this->toDeleteModels.end(); miter++)
    {
      //std::cerr << "ProcessEngitiesToDelete " <<  (*miter)->GetName() << std::endl;
      (*miter)->Fini();
      this->models.erase(
          std::remove(this->models.begin(), this->models.end(), *miter) );
      delete (*miter);
    }

    this->toDeleteModels.clear();
  }
 
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
  //this->SetModelPose(model, model->GetInitPose());

  // Add the model to our list
  this->models.push_back(model);

  if (Simulator::Instance()->GetSimTime() > 0)
    model->Init();

  if (parent != NULL)
    model->Attach(node->GetChild("attach"));

  return model;
}


////////////////////////////////////////////////////////////////////////////////
// Set the model pose and the pose of it's attached children
void World::SetModelPose(Model *model , Pose3d pose)
{
  /*printf("World::SetModelPose\n");
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
  */
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
// Register a geom
void World::RegisterGeom(Geom *geom)
{
  this->geometries.push_back(geom);
}

////////////////////////////////////////////////////////////////////////////////
// Register a body
void World::RegisterBody( Body *body )
{
  this->bodies.push_back(body);
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
  this->showBoundingBoxesSignal(this->showBoundingBoxes);
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
  this->showJointsSignal(show);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the joints
void World::SetShowContacts(bool show)
{
  this->showContacts = show;
  this->showContactsSignal(this->showContacts);
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether to show the joints
bool World::GetShowContacts() const
{
  return this->showContacts;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the light source visuals
void World::SetShowLights(bool show)
{
  this->showLights = show;
  this->showLightsSignal(this->showLights);
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether to show the light source visuals
bool World::GetShowLights() const
{
  return this->showLights;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the camera visuals
void World::SetShowCameras(bool show)
{
  this->showCameras = show;
  this->showCamerasSignal(this->showCameras);
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether to show the camera visuals
bool World::GetShowCameras() const
{
  return this->showCameras;
}

////////////////////////////////////////////////////////////////////////////////
/// Set to view as wireframe
void World::SetWireframe( bool wire )
{
  this->wireframe = wire;
  this->wireframeSignal(this->wireframe);
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
  this->showPhysicsSignal(this->showPhysics);
}

////////////////////////////////////////////////////////////////////////////////
/// Set to use perpixel lighting or pervertex lighting
void World::SetPerPixelLighting( bool pp )
{
  this->perPixelLighting = pp;
  this->perPixelLightingSignal(pp);
}

////////////////////////////////////////////////////////////////////////////////
/// Get to use perpixel lighting or pervertex lighting
bool World::GetPerPixelLighting()
{
  return this->perPixelLighting;
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

  this->simIface->data->simTime = Simulator::Instance()->GetSimTime().Double();
  this->simIface->data->pauseTime = Simulator::Instance()->GetPauseTime().Double();
  this->simIface->data->realTime = Simulator::Instance()->GetRealTime().Double();
  this->simIface->data->state = !Simulator::Instance()->IsPaused();

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
      case SimulationRequestData::UNPAUSE: 
        Simulator::Instance()->SetPaused(false);
        break;
      case SimulationRequestData::PAUSE: 
        Simulator::Instance()->SetPaused(
            !Simulator::Instance()->IsPaused());
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
            model->SetAbsPose(pose);

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
            model->SetAbsPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Set Pose 3d Request.\n";
          }

          break;
        }

      case SimulationRequestData::GET_NUM_MODELS:
        {
          response->type= req->type;
          response->uintValue = this->models.size();
          response++;
          this->simIface->data->responseCount += 1;
          break;
        }

      case SimulationRequestData::GET_NUM_CHILDREN:
        {
          Model *model = this->GetModelByName((char*)req->modelName);

          if (model)
          {
            response->type= req->type;
            response->uintValue = model->GetChildren().size();
            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Num Children.\n";
          break;
        }

      case SimulationRequestData::GET_MODEL_NAME:
        {
          unsigned int index = req->uintValue;

          if (index < this->models.size())
          {
            Model *model = this->models[index];
            memset(response->modelName, 0, 512);

            strncpy(response->modelName, model->GetName().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Model Name.\n";

          break;
        }

      case SimulationRequestData::GET_CHILD_NAME:
        {
          Model *model = this->GetModelByName((char*)req->modelName);

          if (model)
          {
            Entity *ent;
            unsigned int index;
            response->type= req->type;

            index = req->uintValue;

            ent = model->GetChildren()[index];
            if (ent)
            {
              memset(response->strValue, 0, 512);
              strncpy(response->modelName, ent->GetName().c_str(), 512);
              response->strValue[511] = '\0';

              response++;
              this->simIface->data->responseCount += 1;
            }
            else
            gzerr(0) << "Invalid child  index in simulation interface Get Num Children.\n";
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Num Children.\n";

          break;
        }

      case SimulationRequestData::GET_MODEL_FIDUCIAL_ID:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            response->type = req->type;
            response->uintValue = model->GetLaserFiducialId();
            response++;
            this->simIface->data->responseCount += 1;
            break;
          } 
        }
      case SimulationRequestData::GET_MODEL_TYPE:
        {
          Model *model = this->GetModelByName((char*)req->modelName);

          if (model)
          {
            response->type = req->type;
            memset(response->strValue, 0, 512);
            strncpy(response->strValue, model->GetType().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Model Type.\n";
          break;
        }

      case SimulationRequestData::GET_MODEL_EXTENT:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Vector3 min, max;
            model->GetBoundingBox(min, max);

            response->type = req->type;
            strcpy( response->modelName, req->modelName);
            response->vec3Value.x = max.x - min.x;
            response->vec3Value.y = max.y - min.y;
            response->vec3Value.z = max.z - min.z;

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Model Extent.\n";

          break;
        }

      case SimulationRequestData::GET_STATE:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Pose3d pose;
            Vector3 linearVel;
            Vector3 angularVel;
            Vector3 linearAccel;
            Vector3 angularAccel;

            pose = model->GetAbsPose();

            // Get the model's linear and angular velocity
            linearVel = model->GetLinearVel();
            angularVel = model->GetAngularVel();

            // Get the model's linear and angular acceleration
            linearAccel = model->GetLinearAccel();
            angularAccel = model->GetAngularAccel();

            response->modelPose.pos.x = pose.pos.x;
            response->modelPose.pos.y = pose.pos.y;
            response->modelPose.pos.z = pose.pos.z;

            response->modelPose.roll = pose.rot.GetAsEuler().x;
            response->modelPose.pitch = pose.rot.GetAsEuler().y;
            response->modelPose.yaw = pose.rot.GetAsEuler().z;

            response->modelLinearVel.x = linearVel.x;
            response->modelLinearVel.y = linearVel.y;
            response->modelLinearVel.z = linearVel.z;

            response->modelAngularVel.x = angularVel.x;
            response->modelAngularVel.y = angularVel.y;
            response->modelAngularVel.z = angularVel.z;

            response->modelLinearAccel.x = linearAccel.x;
            response->modelLinearAccel.y = linearAccel.y;
            response->modelLinearAccel.z = linearAccel.z;

            response->modelAngularAccel.x = angularAccel.x;
            response->modelAngularAccel.y = angularAccel.y;
            response->modelAngularAccel.z = angularAccel.z;

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get State Request.\n";
          break;
        }
 
      case SimulationRequestData::GET_POSE2D:
      case SimulationRequestData::GET_POSE3D:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Pose3d pose = model->GetAbsPose();
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

      case SimulationRequestData::GET_INTERFACE_TYPE:
        {
          //printf("Model Type Request\n");
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->modelName, req->modelName);
          std::vector<Model*>::iterator mmiter;

          for (mmiter=models.begin(); mmiter!=models.end(); mmiter++)
            GetInterfaceNames((*mmiter), list);

          std::string mname = req->modelName;		
          unsigned int i=mname.find(".");        
/*

		unsigned int ind = list[j].find(mname);
		if(ind==0 && ind!=std::string::npos && list[j].size() > mname.size()){
			candids.push_back(list[j].substr(ind+mname.size(),list[j].size()-ind-mname.size()));
		}
	  }
*/
          while(i!= std::string::npos)
          {
            mname.erase(i,1);
            mname.insert(i,"::");
            i= mname.find(".");
          }


          std::vector<std::string> candids;

          for(unsigned int j=0;j<list.size();j++)
          {
            size_t ind = list[j].find(mname);
            if( ind==0 && ind != std::string::npos && 
               list[j].size() > mname.size())
            {
              candids.push_back(list[j].substr(ind+mname.size(),
                    list[j].size()-ind-mname.size()) );
            }
          }

          /*for(unsigned int ii=0;ii<candids.size();ii++)
            printf("candidatetypes: %s\n",candids[ii].c_str());*/

          for(i=0; i<candids.size(); i++)
          {
            if(candids[i][0]=='>')
            {
              strcpy(response->strValue,
                     candids[i].substr(2,candids[i].size()-2).c_str());
              response->strValue[511]='\0';
              i=candids.size()+5;
            }
          }

          if(strcmp(response->strValue,"irarray")==0)
          {
            strcpy(response->strValue,"ranger");
            response->strValue[511]='\0';		
          }

          if(i<candids.size()+4) // the model is not an interface
          {
            strcpy(response->strValue,"unkown");
            response->strValue[511]='\0';
          }

          //printf("-> modeltype: %s \n", response->modelType);

          response++;
          this->simIface->data->responseCount += 1;

          break;
        }

      case SimulationRequestData::GET_MODEL_INTERFACES:
        {
          response->nChildInterfaces=0;
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->modelName, req->modelName);
          std::vector<Model*>::iterator mmiter;


   	  for (mmiter=models.begin(); mmiter!=models.end(); mmiter++)
  	      	  GetInterfaceNames((*mmiter), list);


	  // removing the ">>type" from the end of each interface names 
	  for(unsigned int jj=0;jj<list.size();jj++){
		unsigned int index = list[jj].find(">>");
		if(index !=std::string::npos)
			list[jj].replace(index,list[jj].size(),"");
	 	//printf("-->> %s \n",list[jj].c_str()); 
	  }
	  
	  
	  //if(strcmp((char*)req->modelName,"")==0){
		  /*
		     for (miter=models.begin(); miter!=models.end(); miter++)
            GetInterfaceNames((*miter), list);
*/

          // removing the ">>type" from the end of each interface names 
          for(unsigned int jj=0;jj<list.size();jj++)
          {
            unsigned int index = list[jj].find(">>");
            if(index !=std::string::npos)
              list[jj].replace(index,list[jj].size(),"");
          }

          if(strcmp((char*)req->modelName,"")==0)
          {
            std::vector<std::string> chlist;
            for(unsigned int i=0;i<list.size();i++)
            {



              		std::string str = list[i].substr(0,list[i].find("::"));
              		std::vector<std::string>::iterator itr;
              		itr = std::find(chlist.begin(),chlist.end(), str);

              		if(itr!=chlist.end() || str=="")
                		continue;

	  		unsigned int ii=str.find("::");        
  	  		while(ii!= std::string::npos){
	
	 			str.erase(ii,2);
  				str.insert(ii,".");
  				ii= str.find("::");
  	  		}

			chlist.push_back(str);
			strcpy(response->childInterfaces[response->nChildInterfaces++],str.c_str());
      			response->childInterfaces[response->nChildInterfaces-1][511]='\0';
		
	    }
  		
          }
          else
          {
            std::vector<std::string> newlist;
            std::string mname = (char*)req->modelName;

            size_t i=mname.find(".");        
            while( i != std::string::npos)
            {
              mname.erase(i,1);
              mname.insert(i,"::");
              i= mname.find(".");
            }

            for(unsigned int j=0;j<list.size();j++)
            {
              unsigned int ind = list[j].find(mname);
              if(ind==0 && ind!=std::string::npos && 
                  list[j].size() > mname.size())
              {
                newlist.push_back(list[j].substr(ind+mname.size()+2,
                      list[j].size()-ind-mname.size()-2));
              }
            }

            std::vector<std::string> chlist;
            for( i=0;i<newlist.size();i++)
            {
              unsigned int indx = newlist[i].find("::");
              indx = (indx==std::string::npos)?newlist[i].size():indx;
              std::string str = newlist[i].substr(0,indx);
              std::vector<std::string>::iterator itr;
              itr = std::find(chlist.begin(),chlist.end(), str);


              if(itr!=chlist.end() || str=="")
                continue;

              chlist.push_back(str);
              // Adding the parent name to the child name e.g "parent.child" 
              str=mname+"."+str;
 	      
	      unsigned int i=str.find("::");        
  	      while(i!=std::string::npos){
			str.erase(i,2);
  			str.insert(i,".");
  			i= str.find("::");
  	      }

              strcpy(response->childInterfaces[response->nChildInterfaces++],
                  str.c_str());
              response->childInterfaces[response->nChildInterfaces-1][511]='\0';
            }
          }

          response++;
          this->simIface->data->responseCount += 1;

          break;  
        }
	

      case SimulationRequestData::GO:
        {
          this->simPauseTime = Simulator::Instance()->GetSimTime() 
                                  + Time(req->runTime);

          Simulator::Instance()->SetPaused(false);
          break;
        }

      case SimulationRequestData::SET_POSE2D:
        {
          Model *model = this->GetModelByName((char*)req->modelName);
          if (model)
          {
            Pose3d pose = model->GetAbsPose();
            Vector3 rot = pose.rot.GetAsEuler();

            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;

            pose.rot.SetFromEuler(Vector3(rot.x, rot.y,
                  req->modelPose.yaw));
            model->SetAbsPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->modelName << "] in simulation interface Get Children Request.\n";
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

  // Remove and delete all models that are marked for deletion
  std::vector< Model* >::iterator miter;
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

void World::GetInterfaceNames(Entity* en, std::vector<std::string>& list)
{
	Model* m = dynamic_cast<Model*>(en);

	if(m)
		m->GetModelInterfaceNames(list);
	
	std::vector<Entity*>::iterator citer;
	for (citer=en->GetChildren().begin(); citer!=en->GetChildren().end(); citer++)
		this->GetInterfaceNames((*citer),list);
}

////////////////////////////////////////////////////////////////////////////////
// Save the state of the world
void World::SaveState()
{
  std::vector<Model*>::iterator mIter;
  std::vector<Body*>::iterator bIter;
  std::vector<Geom*>::iterator gIter;

  WorldState *ws = &(*this->worldStatesInsertIter);
  for (mIter = this->models.begin(); mIter != this->models.end(); mIter++)
    ws->modelPoses[(*mIter)->GetName()] = (*mIter)->GetRelativePose();

  for (bIter = this->bodies.begin(); bIter !=this->bodies.end(); bIter++)
    ws->bodyPoses[(*bIter)->GetName()] = (*bIter)->GetRelativePose();

  for (gIter = this->geometries.begin(); gIter !=this->geometries.end(); 
       gIter++)
    ws->geomPoses[(*gIter)->GetName()] = (*gIter)->GetRelativePose();

  this->worldStatesInsertIter++;
  if (this->worldStatesInsertIter == this->worldStates.end())
    this->worldStatesInsertIter = this->worldStates.begin();

  if (this->worldStatesInsertIter == this->worldStatesEndIter)
  {
    this->worldStatesEndIter++;
    if (this->worldStatesEndIter == this->worldStates.end())
    {
      this->worldStatesEndIter = this->worldStates.begin();
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Set the state of the world to the pos pointed to by the iterator
void World::SetState(std::deque<WorldState>::iterator iter)
{
  std::vector<Model*>::iterator mIter;
  std::vector<Body*>::iterator bIter;
  std::vector<Geom*>::iterator gIter;

  WorldState *ws = &(*iter);
  for (mIter = this->models.begin(); mIter != this->models.end(); mIter++)
    (*mIter)->SetRelativePose( ws->modelPoses[(*mIter)->GetName()] );

  for (bIter = this->bodies.begin(); bIter !=this->bodies.end(); bIter++)
    (*bIter)->SetRelativePose( ws->bodyPoses[(*bIter)->GetName()] );

  for (gIter = this->geometries.begin(); gIter !=this->geometries.end(); 
       gIter++)
    (*gIter)->SetRelativePose( ws->geomPoses[(*gIter)->GetName()] );

  this->worldStatesCurrentIter = iter;
}


////////////////////////////////////////////////////////////////////////////////
/// Goto a position in time
void World::GotoTime(double pos)
{
  Simulator::Instance()->SetPaused(true);

  this->worldStatesCurrentIter = this->worldStatesInsertIter;

  int diff = this->worldStatesInsertIter - this->worldStatesEndIter;

  int i = (int)(diff * (1.0-pos));

  if (this->worldStatesCurrentIter == this->worldStates.begin())
    this->worldStatesCurrentIter = this->worldStates.end()--;

  for (;i>=0; i--, this->worldStatesCurrentIter--)
  {
    if (this->worldStatesCurrentIter == this->worldStatesEndIter)
      break;

    if (this->worldStatesCurrentIter == this->worldStates.begin())
      this->worldStatesCurrentIter = this->worldStates.end()-1;
  }

  this->SetState(this->worldStatesCurrentIter);
}

////////////////////////////////////////////////////////////////////////////////
// Pause callback
void World::PauseSlot(bool p)
{
  if (!p)
    this->worldStatesInsertIter = this->worldStatesCurrentIter;
}
