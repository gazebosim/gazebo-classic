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

#include "OgreVisual.hh"
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
#include "gz.h"
#include "World.hh"
#include "Logger.hh"

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
  this->physicsEngine = NULL;
  this->server = NULL;
  this->graphics = NULL;
  this->openAL = NULL;
  this->selectedEntity = NULL;

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
  // Clear out the entity tree
  std::vector<Model*>::iterator iter;
  for (iter = this->models.begin(); iter != this->models.end(); iter++)
  {
    if (*iter)
    {
      (*iter)->Fini();
      delete *iter;
      (*iter) = NULL;
    }
  }

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
  this->server = new libgazebo::Server();

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
    this->simIface = new libgazebo::SimulationIface();
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
  if (rootNode && rootNode->GetChild("openal","audio"))
  {
    this->openAL = OpenAL::Instance();
    this->openAL->Load(rootNode->GetChild("openal", "audio"));
  }

  XMLConfigNode *physicsNode = NULL;
  if (rootNode )
    physicsNode = rootNode->GetChildByNSPrefix("physics");

  if (Simulator::Instance()->GetPhysicsEnabled() && physicsNode)
  {
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( physicsNode->GetName());
    if (this->physicsEngine == NULL)
      gzthrow("Unable to create physics engine\n");
  }
  else
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine("ode");

  // This should come before loading of entities
  this->physicsEngine->Load(rootNode);
  
  // last bool is initModel, init model is not needed as Init()
  // is called separately from main.cc
  this->LoadEntities(rootNode, NULL, false, false);

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

  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    if (*miter)
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
#ifdef USE_THREADPOOL
  // If calling Body::Update in threadpool
  this->physicsEngine->InitForThread();
#endif

  std::vector< Model* >::iterator miter;

  this->simPauseTime = 0;

  // Initialize all the entities
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    if (*miter)
      (*miter)->Init();
  }

  // Initialize the physics engine
  this->physicsEngine->Init();

  // Initialize openal
  if (this->openAL)
    this->openAL->Init();

  this->toDeleteEntities.clear();
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
  std::vector< Model* >::const_iterator miter;
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
  this->worldUpdateStartSignal();

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
    DIAGNOSTICTIMER(timer("World::Update Models",6));

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

  if (!Simulator::Instance()->IsPaused() &&
       Simulator::Instance()->GetPhysicsEnabled())
  {
    {
      DIAGNOSTICTIMER(timer("World::Update Physics",6));
      this->physicsEngine->UpdatePhysics();
    }

    /*if (this->saveStateTimer.GetElapsed() > **this->saveStateTimeoutP)
    {
      this->SaveState();
      this->saveStateTimer.Start();
    }*/
  }

  /// Update all the sensors
  SensorManager::Instance()->Update();


  this->factory->Update();

  Logger::Instance()->Update();

  this->worldUpdateEndSignal();
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
libgazebo::Server *World::GetGzServer() const
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
void World::LoadEntities(XMLConfigNode *node, Model *parent, bool removeDuplicate, bool initModel)
{
  XMLConfigNode *cnode;
  Model *model = NULL;

  if (node && node->GetNSPrefix() != "")
  {
    // Check for model nodes
    if (node->GetNSPrefix() == "model")
    {
      model = this->LoadModel(node, parent, removeDuplicate, initModel);
      this->addEntitySignal(model);
    }
  }

  // Load children
  if (node)
    for (cnode = node->GetChild(); cnode != NULL; cnode = cnode->GetNext())
      this->LoadEntities( cnode, model, removeDuplicate, initModel);
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

      // last bool is initModel, yes, init model after loading it
      this->LoadEntities( xmlConfig->GetRootNode(), NULL, true, true); 
      delete xmlConfig;
    }
    this->toLoadEntities.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load all the entities that have been queued
void World::ProcessEntitiesToDelete()
{
  if (!this->toDeleteEntities.empty())
  {
    // maybe try try_lock here instead
    boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMDMutex());

    // Remove and delete all models that are marked for deletion
    std::vector< std::string>::iterator miter;
    for (miter=this->toDeleteEntities.begin();
        miter!=this->toDeleteEntities.end(); miter++)
    {
      Entity *entity = this->GetEntityByName(*miter);

      if (entity)
      {
        if (entity->GetType() == Entity::MODEL)
        {
          Model *model = (Model*)entity;

          model->Fini();

          std::vector<Model*>::iterator newiter;
          newiter = std::find(this->models.begin(), this->models.end(), model);

          if (newiter != this->models.end())
            this->models.erase( newiter );
        }
        else if (entity->GetType() == Entity::BODY)
          ((Body*)entity)->Fini();

        delete (entity);
        this->deleteEntitySignal(*miter);
      }
    }

    this->toDeleteEntities.clear();
  }
 
}

////////////////////////////////////////////////////////////////////////////////
/// Delete an entity by name
void World::DeleteEntity(const std::string &name)
{
  std::vector< Model* >::iterator miter;

  Entity *entity = this->GetEntityByName(name);

  if (!entity)
    return;

  this->toDeleteEntities.push_back(name);
}


////////////////////////////////////////////////////////////////////////////////
// Load a model
Model *World::LoadModel(XMLConfigNode *node, Model *parent, 
                        bool removeDuplicate, bool initModel)
{
  Pose3d pose;
  Model *model = new Model(parent);

  // Load the model
  model->Load( node, removeDuplicate );

  // If calling LoadEntity()->LoadModel()from Simulator::Load()->World::Load()
  // GetWorldInitialized() is false, in this case, model->Init() is
  // called later directly from main.cc through Simulator::Init()
  // on the other hand
  // LoadEntity()->LoadModel() is also called from ProcessEntitesToLoad(),
  // in this case, GetWorldInitialized should return true, and we want
  // to call model->Init() here
  if (initModel) 
    model->Init();

  if (parent != NULL)
    model->Attach(node->GetChild("attach"));
  else
    // Add the model to our list
    this->models.push_back(model);

  return model;
}

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to a model based on a name
Entity *World::GetEntityByName(const std::string &name) const
{
  std::vector< Model *>::const_iterator iter;
  Entity *result = NULL;

  for (iter = this->models.begin(); 
       iter != this->models.end() && result == NULL; iter++)
    result = this->GetEntityByNameHelper(name, (*iter));
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Get an entity by name
Entity *World::GetEntityByNameHelper(const std::string &name, Entity *parent) const
{
  if (!parent)
    return NULL;

  if (parent->GetCompleteScopedName() == name)
    return parent;

  const std::vector<Entity*> children = parent->GetChildren();
  std::vector< Entity* >::const_iterator iter;

  Entity *result = NULL;

  for (iter = children.begin(); iter != children.end() && result ==NULL; iter++)
    result = this->GetEntityByNameHelper(name, *iter);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
///  Get an iterator over the models
const std::vector<Model*> &World::GetModels() const
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
// Update the simulation interface
void World::UpdateSimulationIface()
{
  libgazebo::SimulationRequestData *response = NULL;

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
  this->simIface->data->stepTime = this->physicsEngine->GetStepTime().Double();
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
    libgazebo::SimulationRequestData *req = &(this->simIface->data->requests[i]);

    switch (req->type)
    {
      case libgazebo::SimulationRequestData::UNPAUSE: 
        Simulator::Instance()->SetPaused(false);
        break;
      case libgazebo::SimulationRequestData::PAUSE: 
        Simulator::Instance()->SetPaused(
            !Simulator::Instance()->IsPaused());
        break;

      case libgazebo::SimulationRequestData::STEP: 
        Simulator::Instance()->SetStepInc(true);
        break;

      case libgazebo::SimulationRequestData::RESET:
        this->Reset();
        break;

      case libgazebo::SimulationRequestData::SAVE:
        Simulator::Instance()->Save();
        break;

      case libgazebo::SimulationRequestData::SET_ENTITY_PARAM_VALUE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent)
            ent->SetParam( req->strValue, req->strValue1 );
          else
            gzerr(0) << "Invalid entity name[" << req->name 
                     << "] in simulation interface Set Param Request.\n";
          break;
        }

      case libgazebo::SimulationRequestData::APPLY_FORCE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::BODY)
          {
            Body *body = (Body*)(ent);

            Vector3 force(req->vec3Value.x, req->vec3Value.y, req->vec3Value.z);

            body->SetForce(force);
          }
          else
          {
            gzerr(0) << "Invalid body name[" << req->name 
              << "] in simulation interface Set Force Request.\n";
          }
          break;

        }

      case libgazebo::SimulationRequestData::APPLY_TORQUE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::BODY)
          {
            Body *body = (Body*)(ent);

            Vector3 torque(req->vec3Value.x,req->vec3Value.y, req->vec3Value.z);

            body->SetTorque(torque);
          }
          else
          {
            gzerr(0) << "Invalid body name[" << req->name 
              << "] in simulation interface Set Torque Request.\n";
          }
          break;

        }
      case libgazebo::SimulationRequestData::SET_LINEAR_VEL:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)(ent);

            Vector3 linearVel( req->modelLinearVel.x, req->modelLinearVel.y,
                               req->modelLinearVel.z);

            Pose3d pose = model->GetWorldPose();
            linearVel = pose.rot.RotateVector(linearVel);
            model->SetLinearVel(linearVel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Vel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_ANGULAR_VEL:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && (ent->GetType() == Entity::MODEL || 
              ent->GetType() == Entity::BODY))
          {

            Vector3 vel( req->modelAngularVel.x, req->modelAngularVel.y,
                         req->modelAngularVel.z);

            Pose3d pose = ent->GetWorldPose();
            vel = pose.rot.RotateVector(vel);

            if (ent->GetType()==Entity::MODEL)
              ((Model*)ent)->SetAngularVel(vel);
            else if (ent->GetType() == Entity::BODY)
              ((Body*)ent)->SetAngularVel(vel);

          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Angular Vel Request.\n";
          }
          break;
        }

 
      case libgazebo::SimulationRequestData::SET_LINEAR_ACCEL:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;

            Vector3 accel( req->modelLinearAccel.x, req->modelLinearAccel.y,
                               req->modelLinearAccel.z);

            Pose3d pose = model->GetWorldPose();
            accel = pose.rot.RotateVector(accel);
            model->SetLinearAccel(accel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Accel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_ANGULAR_ACCEL:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;
            Vector3 accel( req->modelAngularAccel.x, req->modelAngularAccel.y,
                               req->modelAngularAccel.z);

            Pose3d pose = model->GetWorldPose();
            accel = pose.rot.RotateVector(accel);
            model->SetAngularAccel(accel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Accel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_STATE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;

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
            model->SetWorldPose(pose);

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
            gzerr(0) << "Invalid model name[" << req->name 
                     << "] in simulation interface Set State Request.\n";
          }
          break;
        }
      case libgazebo::SimulationRequestData::SET_POSE3D:
        {
          Pose3d pose;
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)(ent);
            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;
            pose.pos.z = req->modelPose.pos.z;

            pose.rot.SetFromEuler(
                Vector3(req->modelPose.roll, 
                  req->modelPose.pitch,
                  req->modelPose.yaw));
            model->SetWorldPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Set Pose 3d Request.\n";
          }

          break;
        }

      case libgazebo::SimulationRequestData::GET_NUM_MODELS:
        {
          response->type= req->type;
          response->uintValue = this->models.size();
          response++;
          this->simIface->data->responseCount += 1;
          break;
        }

      case libgazebo::SimulationRequestData::GET_NUM_CHILDREN:
        {
          Entity *entity = this->GetEntityByName((char*)req->name);

          if (entity)
          {
            response->type= req->type;
            response->uintValue = entity->GetChildren().size();
            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface Get Num Children.\n";
          break;
        }

      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_KEY:
        {
          Entity *entity = this->GetEntityByName((char*)req->name);
          if (entity)
          {
            Param *param = entity->GetParam(req->uintValue);
            response->type= req->type;
            memset(response->strValue, 0, 512);

            if (param)
              strncpy(response->strValue, param->GetKey().c_str(), 512);

            response->strValue[511] = '\0';

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_VALUE:
        {
          Entity *entity = this->GetEntityByName((char*)req->name);
          if (entity)
          {
            Param *param = entity->GetParam(req->uintValue);
            response->type= req->type;
            memset(response->strValue, 0, 512);

            if (param)
              strncpy(response->strValue, param->GetAsString().c_str(), 512);

            response->strValue[511] = '\0';
            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM.\n";

          break;
        }



      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_COUNT:
        {
          Entity *entity = this->GetEntityByName((char*)req->name);
          if (entity)
          {
            unsigned int count = entity->GetParamCount();
            response->type= req->type;
            response->uintValue = count;
            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM_COUNT.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_NAME:
        {
          unsigned int index = req->uintValue;

          if (index < this->models.size())
          {
            Model *model = this->models[index];
            memset(response->name, 0, 512);

            strncpy(response->name, model->GetCompleteScopedName().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Name.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_CHILD_NAME:
        {
          Entity *entity = this->GetEntityByName((char*)req->name);

          if (entity)
          {
            Entity *child;
            unsigned int index;
            response->type= req->type;

            index = req->uintValue;

            child = entity->GetChildren()[index];
            if (child)
            {
              memset(response->strValue, 0, 512);
              strncpy(response->name, child->GetCompleteScopedName().c_str(), 512);
              response->strValue[511] = '\0';

              response++;
              this->simIface->data->responseCount += 1;
            }
            else
            gzerr(0) << "Invalid child index in simulation interface Get Child Name.\n";
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Child Name.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_FIDUCIAL_ID:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;
            response->type = req->type;
            response->uintValue = model->GetLaserFiducialId();
            response++;
            this->simIface->data->responseCount += 1;
            break;
          } 
        }
      case libgazebo::SimulationRequestData::GET_ENTITY_TYPE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);
          if (ent)
          {
            response->type = req->type;
            memset(response->strValue, 0, 512);
            if (ent->GetType() == Entity::MODEL)
              strncpy(response->strValue, "model", 512);
            else if (ent->GetType() == Entity::BODY)
              strncpy(response->strValue, "body", 512);
            else if (ent->GetType() == Entity::GEOM)
              strncpy(response->strValue, "geom", 512);

            response->strValue[511] = '\0';
            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface Get Model Type.\n";

          break;
        }
      case libgazebo::SimulationRequestData::GET_MODEL_TYPE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;

            response->type = req->type;
            memset(response->strValue, 0, 512);
            strncpy(response->strValue, model->GetModelType().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Type.\n";
          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_EXTENT:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);
          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;
            Vector3 min, max;
            model->GetBoundingBox(min, max);

            response->type = req->type;
            strcpy( response->name, req->name);
            response->vec3Value.x = max.x - min.x;
            response->vec3Value.y = max.y - min.y;
            response->vec3Value.z = max.z - min.z;

            response++;
            this->simIface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Extent.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_STATE:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);
          if (ent)// && ent->GetType() == Entity::MODEL)
          {
            Pose3d pose;
            Vector3 linearVel;
            Vector3 angularVel;
            Vector3 linearAccel;
            Vector3 angularAccel;

            pose = ent->GetWorldPose();

            // Get the model's linear and angular velocity
            linearVel = ent->GetWorldLinearVel();
            angularVel = ent->GetWorldAngularVel();

            // Get the model's linear and angular acceleration
            linearAccel = ent->GetWorldLinearAccel();
            angularAccel = ent->GetWorldAngularAccel();

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
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get State Request.\n";
          break;
        }
 
      case libgazebo::SimulationRequestData::GET_POSE2D:
      case libgazebo::SimulationRequestData::GET_POSE3D:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);
          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;

            Pose3d pose = model->GetWorldPose();
            Vector3 rot = pose.rot.GetAsEuler();

            response->type = req->type;

            strcpy( response->name, req->name);
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
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Pose 3d Request.\n";
          }

          break;
        }

      case libgazebo::SimulationRequestData::GET_INTERFACE_TYPE:
        {
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->name, req->name);
          std::vector<Model*>::iterator mmiter;

          for (mmiter=models.begin(); mmiter!=models.end(); mmiter++)
            GetInterfaceNames((*mmiter), list);

          std::string mname = req->name;		
          unsigned int i=mname.find(".");        

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

          response++;
          this->simIface->data->responseCount += 1;

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_INTERFACES:
        {
          response->nChildInterfaces=0;
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->name, req->name);
          std::vector<Model*>::iterator mmiter;


          for (mmiter=models.begin(); mmiter!=models.end(); mmiter++)
            GetInterfaceNames((*mmiter), list);


          // removing the ">>type" from the end of each interface names 
          for(unsigned int jj=0;jj<list.size();jj++){
            unsigned int index = list[jj].find(">>");
            if(index !=std::string::npos)
              list[jj].replace(index,list[jj].size(),"");
          }
	  
          // removing the ">>type" from the end of each interface names 
          for(unsigned int jj=0;jj<list.size();jj++)
          {
            unsigned int index = list[jj].find(">>");
            if(index !=std::string::npos)
              list[jj].replace(index,list[jj].size(),"");
          }

          if(strcmp((char*)req->name,"")==0)
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
            std::string mname = (char*)req->name;

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

     case libgazebo::SimulationRequestData::START_LOG:
        {
          Logger::Instance()->AddLog(req->name, req->strValue);
          break;
        }

     case libgazebo::SimulationRequestData::STOP_LOG:
        {
          Logger::Instance()->RemoveLog(req->name);
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_TIME:
        {
          this->physicsEngine->SetStepTime(Time(req->dblValue));
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_ITERS:
        {
          this->physicsEngine->SetSORPGSIters(req->uintValue);
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_TYPE:
        {
          this->physicsEngine->SetStepType(req->strValue);
          break;
        }

     case libgazebo::SimulationRequestData::GET_STEP_TYPE:
        {
          memset(response->strValue, 0, 512);
          strncpy(response->strValue, this->physicsEngine->GetStepType().c_str(), 512);
          response->strValue[511] = '\0';
          response++;
          this->simIface->data->responseCount += 1;
          break;
        }

     case libgazebo::SimulationRequestData::GET_PLUGIN_COUNT:
        {
          response->type= req->type;
          response->uintValue = Simulator::Instance()->GetPluginCount();
          response++;
          this->simIface->data->responseCount += 1;
          break;
        }

     case libgazebo::SimulationRequestData::GET_PLUGIN_NAME:
        {
          memset(response->strValue, 0, 512);
          strncpy(response->strValue, Simulator::Instance()->GetPluginName(req->uintValue).c_str(), 512);
          response->strValue[511] = '\0';
          response++;
          this->simIface->data->responseCount += 1;
          break;
        }

 
     case libgazebo::SimulationRequestData::ADD_PLUGIN:
        {
          Simulator::Instance()->AddPlugin(req->strValue, req->name);
          break;
        }

     case libgazebo::SimulationRequestData::REMOVE_PLUGIN:
        {
          Simulator::Instance()->RemovePlugin(req->strValue);
          break;
        }

     case libgazebo::SimulationRequestData::GO:
        {
          int sec = req->runTime/1000;
          int nsec = (req->runTime - sec*1000) * 1e6;

          this->simPauseTime = Simulator::Instance()->GetSimTime() 
                                  + Time(sec, nsec);

          Simulator::Instance()->SetPaused(false);
          break;
        }

      case libgazebo::SimulationRequestData::SET_POSE2D:
        {
          Entity *ent = this->GetEntityByName((char*)req->name);

          if (ent && ent->GetType() == Entity::MODEL)
          {
            Model *model = (Model*)ent;

            Pose3d pose = model->GetWorldPose();
            Vector3 rot = pose.rot.GetAsEuler();

            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;

            pose.rot.SetFromEuler(Vector3(rot.x, rot.y,
                  req->modelPose.yaw));
            model->SetWorldPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Children Request.\n";
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
}

////////////////////////////////////////////////////////////////////////////////
// Get all the interface names
void World::GetInterfaceNames(Entity* en, std::vector<std::string>& list)
{
  if (!en || en->GetType() != Entity::MODEL)
    return;

	Model* m = (Model*)(en);

  m->GetModelInterfaceNames(list);

  const std::vector<Entity*> children = en->GetChildren();
	std::vector<Entity*>::const_iterator citer;
	for (citer=children.begin(); citer!=children.end(); citer++)
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

/*  for (bIter = this->bodies.begin(); bIter !=this->bodies.end(); bIter++)
    (*bIter)->SetRelativePose( ws->bodyPoses[(*bIter)->GetName()] );

  for (gIter = this->geometries.begin(); gIter !=this->geometries.end(); 
       gIter++)
    (*gIter)->SetRelativePose( ws->geomPoses[(*gIter)->GetName()] );
    */

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

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void World::SetSelectedEntity( Entity *ent )
{
  // unselect selectedEntity
  if (this->selectedEntity)
  {
    this->selectedEntity->GetVisualNode()->ShowSelectionBox(false);
    this->selectedEntity->SetSelected(false);
  }

  // if a different entity is selected, show bounding box and SetSelected(true)
  if (ent && this->selectedEntity != ent)
  {
    // set selected entity to ent
    this->selectedEntity = ent;
    this->selectedEntity->GetVisualNode()->ShowSelectionBox(true);
    this->selectedEntity->SetSelected(true);
  }
  else
    this->selectedEntity = NULL;

  this->entitySelectedSignal(this->selectedEntity);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the selected entity
Entity *World::GetSelectedEntity() const
{
  return this->selectedEntity;
}

////////////////////////////////////////////////////////////////////////////////
/// Print entity tree
void World::PrintEntityTree()
{
  for (std::vector<Model*>::iterator iter = this->models.begin();
       iter != this->models.end(); iter++)
  {
    (*iter)->Print("");
  }
}
