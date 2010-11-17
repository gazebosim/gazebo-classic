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

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "Events.hh"
#include "Visual.hh"
#include "Body.hh"
#include "FactoryIfaceHandler.hh"
#include "GraphicsIfaceHandler.hh"
#include "SimulationIfaceHandler.hh"
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
#include "Scene.hh"
#include "World.hh"

// NATY: put back in
//#include "Logger.hh"

#include "OpenAL.hh"

#include "Geom.hh"

using namespace gazebo;

class ModelUpdate_TBB
{
  public: ModelUpdate_TBB(std::vector<Model*> *models) : models(models) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      (*models)[i]->Update();
    }
  }

  private: std::vector<Model*> *models;
};

////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  this->server = NULL;
  this->physicsEngine = NULL;
  this->graphics = NULL;
  this->simIfaceHandler = NULL;
  this->openAL = NULL;
  this->selectedEntity = NULL;

  this->factoryIfaceHandler = NULL;
  this->pause = false;

  Param::Begin(&this->parameters);
  this->nameP = new ParamT<std::string>("name","default",1);
  this->saveStateTimeoutP = new ParamT<Time>("saveStateResolution",0.1,0);
  this->saveStateBufferSizeP = new ParamT<unsigned int>("saveStateBufferSize",1000,0);
  Param::End();

  this->scene = new Scene("scene");
  this->scene->SetType(Scene::GENERIC);
  this->scene->SetAmbientColor(Color(0.5, 0.5, 0.5));
  this->scene->SetBackgroundColor(Color(0.5, 0.5, 0.5, 1.0));
  this->scene->CreateGrid( 10, 1, 0.03, Color(1,1,1,1));
  this->scene->Init();

  Events::ConnectPauseSignal( boost::bind(&World::PauseSlot, this, _1) );
  Events::ConnectSetSelectedEntitySignal( boost::bind(&World::SetSelectedEntityCB, this, _1) );
  Events::ConnectDeleteEntitySignal( boost::bind(&World::DeleteEntityCB, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  delete this->nameP;
  delete this->saveStateTimeoutP;
  delete this->saveStateBufferSizeP;

  Events::DisconnectPauseSignal( boost::bind(&World::PauseSlot, this, _1) );
  Events::DisconnectSetSelectedEntitySignal( boost::bind(&World::SetSelectedEntityCB, this, _1) );
  Events::DisconnectDeleteEntitySignal( boost::bind(&World::DeleteEntityCB, this, _1) );

  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(XMLConfigNode *rootNode)//, unsigned int serverId)
{
  this->nameP->Load(rootNode);
  this->saveStateTimeoutP->Load(rootNode);
  this->saveStateBufferSizeP->Load(rootNode);

  // Create the server object (needs to be done before models initialize)
  if (this->server == NULL)
  {
    this->server = new libgazebo::Server();

    try
    {
      this->server->Init(**this->nameP, true );
    }
    catch (std::string err)
    {
      gzthrow (err);
    }
  }

  // Create the simulator interface
  try
  {
    if (!this->simIfaceHandler)
    {
      this->simIfaceHandler = new SimulationIfaceHandler(this);
    }
  }
  catch (std::string err)
  {
    gzthrow(err);
  }

  // Create the default factory
  if (!this->factoryIfaceHandler)
    this->factoryIfaceHandler = new FactoryIfaceHandler(this);

  // Create the graphics iface handler
  if (!this->graphics && Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->graphics = new GraphicsIfaceHandler(this);
    this->graphics->Load("default");
  }

  // Load OpenAL audio 
  if (rootNode && rootNode->GetChild("audio"))
  {
    this->openAL = OpenAL::Instance();
    this->openAL->Load(rootNode->GetChild("audio"));
  }

  XMLConfigNode *physicsNode = NULL;
  if (rootNode )
    physicsNode = rootNode->GetChild("physics");

  if (Simulator::Instance()->GetPhysicsEnabled() && physicsNode)
  {
    if (this->physicsEngine)
    {
      delete this->physicsEngine;
      this->physicsEngine = NULL;
    }
    std::string type = physicsNode->GetString("type","ode",1);

    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( type, this);
    if (this->physicsEngine == NULL)
      gzthrow("Unable to create physics engine\n");
  }
  else
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine("ode", this);

  // This should come before loading of entities
  this->physicsEngine->Load(physicsNode);
  
  // last bool is initModel, init model is not needed as Init()
  // is called separately from main.cc
  this->LoadEntities(rootNode, NULL, false, false);

  this->worldStates.resize(**this->saveStateBufferSizeP);
  this->worldStatesInsertIter = this->worldStates.begin();
  this->worldStatesEndIter = this->worldStates.begin();
  this->worldStatesCurrentIter = this->worldStatesInsertIter;
}

////////////////////////////////////////////////////////////////////////////////
// Save the world
void World::Save(std::string &prefix, std::ostream &stream)
{
  std::vector< Model* >::iterator miter;

  stream << "<world>\n";

  stream << prefix << "  " << *(this->nameP);
  stream << prefix << "  " << *(this->saveStateTimeoutP);
  stream << prefix << "  " << *(this->saveStateBufferSizeP);

  this->physicsEngine->Save(prefix, stream);

  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    if (*miter)
    {
      (*miter)->Save(prefix, stream);
      stream << "\n";
    }
  }

  stream << "</world>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Initialize the world
void World::Init()
{
  std::vector< Model* >::iterator miter;

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

  this->factoryIfaceHandler->Init();
  //this->saveStateTimer.Start();
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
// Run the world in a thread
void World::Start()
{
  this->stop = false;
  this->thread = new boost::thread( 
      boost::bind(&World::RunLoop, this));
}

////////////////////////////////////////////////////////////////////////////////
// Stop the world
void World::Stop()
{
  this->stop = true;
  if (this->thread)
  {
    this->thread->join();
    delete this->thread;
    this->thread = NULL;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Function to run physics. Used by physicsThread
void World::RunLoop()
{
  this->physicsEngine->InitForThread();

  Time step = this->physicsEngine->GetStepTime();
  double physicsUpdateRate = this->physicsEngine->GetUpdateRate();
  Time physicsUpdatePeriod = 1.0 / physicsUpdateRate;

  //bool userStepped;
  Time diffTime;
  Time currTime;
  Time lastTime = this->GetRealTime();
  struct timespec req, rem;

  this->startTime = Time::GetWallTime();

  this->stop = false;
  while (!this->stop)
  {
    lastTime = this->GetRealTime();
    if (this->IsPaused())
      this->pauseTime += step;
    else
    {
      this->simTime += step;
      this->Update();
    }

    currTime = this->GetRealTime();

    // Set a default sleep time
    req.tv_sec  = 0;
    req.tv_nsec = 10000;

    // If the physicsUpdateRate < 0, then we should try to match the
    // update rate to real time
    if ( physicsUpdateRate < 0 &&
        (this->GetSimTime() + this->GetPauseTime()) > 
        this->GetRealTime()) 
    {
      diffTime = (this->GetSimTime() + this->GetPauseTime()) - 
                  this->GetRealTime();
      req.tv_sec  = diffTime.sec;
      req.tv_nsec = diffTime.nsec;
    }
    // Otherwise try to match the update rate to the one specified in
    // the xml file
    else if (physicsUpdateRate > 0 && 
        currTime - lastTime < physicsUpdatePeriod)
    {
      diffTime = physicsUpdatePeriod - (currTime - lastTime);

      req.tv_sec  = diffTime.sec;
      req.tv_nsec = diffTime.nsec;
    }

    //nanosleep(&req, &rem);

    // TODO: Fix timeout:  this belongs in simulator.cc
    /*if (this->timeout > 0 && this->GetRealTime() > this->timeout)
    {
      this->stop = true;
      break;
    }*/

    // TODO: Fix stepping
    /*if (userStepped)
    {
      this->SetStepInc(false);
      this->SetPaused(true);
    }*/
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
void World::Update()
{
  Events::worldUpdateStartSignal();

  tbb::parallel_for( tbb::blocked_range<size_t>(0, this->models.size(), 10),
      ModelUpdate_TBB(&this->models) );

  if (this->physicsEngine)
    this->physicsEngine->UpdatePhysics();

  /// Update all the sensors
  SensorManager::Instance()->Update();

  this->factoryIfaceHandler->Update();

  // Process all incoming messages from simiface
  this->simIfaceHandler->Update();

  // NATY: put back in
  //Logger::Instance()->Update();

  Events::worldUpdateEndSignal();
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
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
  this->models.clear();

  if (this->graphics)
  {
    delete this->graphics;
    this->graphics = NULL;
  }

  if (this->simIfaceHandler)
  {
    delete this->simIfaceHandler;
    this->simIfaceHandler = NULL;
  }

  if (this->factoryIfaceHandler)
  {
    delete this->factoryIfaceHandler;
    this->factoryIfaceHandler = NULL;
  }

  if (this->physicsEngine)
  {
    this->physicsEngine->Fini();
    delete this->physicsEngine;
    this->physicsEngine = NULL;
  }

  try
  {
    if (this->server)
    {
      delete this->server;
      this->server = NULL;
    }
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
/// Remove all entities from the world
void World::Clear()
{
  std::vector<Model*>::iterator iter;
  for (iter = this->models.begin(); iter != this->models.end(); iter++)
    Events::deleteEntitySignal((*iter)->GetCompleteScopedName());
  this->ProcessEntitiesToDelete();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the world
std::string World::GetName() const
{
  return **this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of parameters
unsigned int World::GetParamCount() const
{
  return this->parameters.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param
Param *World::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr(2) << "World::GetParam - Invalid param index\n";

  return NULL;
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

  if (node)
  {
    // Check for model nodes
    if (node->GetName() == "model")
    {
      model = this->LoadModel(node, parent, removeDuplicate, initModel);
      Events::addEntitySignal(model->GetCompleteScopedName());
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
    // NATY
    // maybe try try_lock here instead
    //boost::recursive_mutex::scoped_lock lock(
        //*Simulator::Instance()->GetMRMutex());

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
    // NATY
    // maybe try try_lock here instead
    //boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMDMutex());

    // Remove and delete all models that are marked for deletion
    std::vector< std::string>::iterator miter;
    for (miter=this->toDeleteEntities.begin();
        miter!=this->toDeleteEntities.end(); miter++)
    {
      Common *common = Common::GetByName(*miter);

      if (common)
      {
        if (common->HasType(MODEL))
        {
          Model *model = (Model*)common;

          model->Fini();

          std::vector<Model*>::iterator newiter;
          newiter = std::find(this->models.begin(), this->models.end(), model);

          if (newiter != this->models.end())
            this->models.erase( newiter );
        }
        else if (common->HasType(BODY))
          ((Body*)common)->Fini();

        delete (common);
      }
    }

    this->toDeleteEntities.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Delete an entity by name
void World::DeleteEntityCB(const std::string &name)
{
  std::vector< Model* >::iterator miter;

  Common *common = Common::GetByName(name);

  if (!common)
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
  model->SetWorld(this);

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


///////////////////////////////////////////////////////////////////////////////
/// Get the number of models
unsigned int World::GetModelCount() const
{
  return this->models.size();
}

///////////////////////////////////////////////////////////////////////////////
/// Get a model based on an index
Model *World::GetModel(unsigned int index)
{
  if (index < this->models.size())
    return this->models[index];
  else
    gzerr(2) << "Invalid model index\n";

  return NULL;
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
/*void World::GotoTime(double pos)
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
}*/

////////////////////////////////////////////////////////////////////////////////
// Pause callback
void World::PauseSlot(bool p)
{
  if (!p)
    this->worldStatesInsertIter = this->worldStatesCurrentIter;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void World::SetSelectedEntityCB( const std::string &name )
{
  Common *common = Common::GetByName(name);
  Entity *ent = dynamic_cast<Entity*>(common);

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

  Events::entitySelectedSignal(this->selectedEntity);
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

////////////////////////////////////////////////////////////////////////////////
// Get the simulation time
gazebo::Time World::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the sim time
void World::SetSimTime(Time t)
{
  this->simTime = t;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
gazebo::Time World::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
gazebo::Time World::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
gazebo::Time World::GetRealTime() const
{
  return Time::GetWallTime() - this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
// Return when this simulator is paused
bool World::IsPaused() const
{
  return this->pause;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the simulation is paused
void World::SetPaused(bool p)
{
  if (this->pause == p)
    return;

  Events::pauseSignal(p);
  this->pause = p;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the scene 
Scene *World::GetScene() const
{
  return this->scene;
}
