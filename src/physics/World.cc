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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 */

#include <boost/thread.hpp>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "transport/Transport.hh"

#include "common/Diagnostics.hh"
#include "common/Events.hh"
#include "common/Global.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/XMLConfig.hh"

#include "physics/Body.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Model.hh"
#include "physics/World.hh"

#include "physics/Geom.hh"

using namespace gazebo;
using namespace physics;


class ModelUpdate_TBB
{
  public: ModelUpdate_TBB(Model_V *models) : models(models) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      (*models)[i]->Update();
    }
  }

  private: Model_V *models;
};

////////////////////////////////////////////////////////////////////////////////
// Private constructor
World::World()
{
  this->stepInc = false;
  this->pause = false;

  common::Param::Begin(&this->parameters);
  this->nameP = new common::ParamT<std::string>("name","default",1);
  common::Param::End();

  this->connections.push_back( 
     event::Events::ConnectPauseSignal(boost::bind(&World::PauseCB, this, _1)));
  this->connections.push_back( 
     event::Events::ConnectStepSignal( boost::bind(&World::StepCB, this) ) );
  this->connections.push_back( 
     event::Events::ConnectSetSelectedEntitySignal( boost::bind(&World::SetSelectedEntityCB, this, _1) ) );
  this->connections.push_back(
     event::Events::ConnectDeleteEntitySignal( boost::bind(&World::DeleteEntityCB, this, _1) ) );
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  delete this->nameP;
  this->connections.clear();
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(common::XMLConfigNode *rootNode)//, unsigned int serverId)
{
  // DO THIS FIRST
  this->nameP->Load(rootNode);

  // The period at which statistics about the world are published
  this->statPeriod = common::Time(0,200000000);

  // Set the global topic namespace
  transport::set_topic_namespace(**this->nameP);

  common::Message::Init( this->worldStatsMsg, "statistics" );

  this->sceneSub = transport::subscribe("~/publish_scene", 
                                        &World::PublishScene, this);
  this->scenePub = transport::advertise<msgs::Scene>("~/scene");

  this->statPub = transport::advertise<msgs::WorldStatistics>("~/world_stats");
  this->visPub = transport::advertise<msgs::Visual>("~/visual");
  this->visSub = transport::subscribe("~/visual", &World::VisualLog, this);

  this->selectionPub = transport::advertise<msgs::Selection>("~/selection");
  this->lightPub = transport::advertise<msgs::Light>("~/light");

  msgs::Scene scene = common::Message::SceneFromXML( rootNode->GetChild("scene") );
  this->scenePub->Publish( scene );

  common::XMLConfigNode *physicsNode = NULL;
  if (rootNode )
    physicsNode = rootNode->GetChild("physics");

  if (physicsNode)
  {
    std::string type = physicsNode->GetString("type","ode",1);

    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( type, shared_from_this());
    if (this->physicsEngine == NULL)
      gzthrow("Unable to create physics engine\n");
  }
  else
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine("ode", shared_from_this());

  // This should come before loading of entities
  this->physicsEngine->Load(physicsNode);

  this->rootElement.reset(new Base(BasePtr()));
  this->rootElement->SetName("root");
  this->rootElement->SetWorld(shared_from_this());

  // First, create all the entities
  this->LoadEntities(rootNode, this->rootElement);

  // Choose threaded or unthreaded model updating depending on the number of
  // models in the scene
  if (this->models.size() < 20)
    this->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  else
    this->modelUpdateFunc = &World::ModelUpdateTBB;
}

////////////////////////////////////////////////////////////////////////////////
// Save the world
void World::Save(std::string &prefix, std::ostream &stream)
{
  Model_V::iterator miter;

  stream << "<world>\n";

  stream << prefix << "  " << *(this->nameP);

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
  Model_V::iterator miter;

  // Initialize all the entities
  for (miter = this->models.begin(); miter != this->models.end(); miter++)
  {
    (*miter)->Init();
  }

  // Initialize the physics engine
  this->physicsEngine->Init();
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

  common::Time step = this->physicsEngine->GetStepTime();
  double physicsUpdateRate = this->physicsEngine->GetUpdateRate();
  common::Time physicsUpdatePeriod = 1.0 / physicsUpdateRate;

  //bool userStepped;
  common::Time diffTime;
  common::Time currTime;
  common::Time lastTime = this->GetRealTime();
  struct timespec req, rem;

  this->startTime = common::Time::GetWallTime();

  this->stop = false;

  // Set a default sleep time
  req.tv_sec  = 0;
  req.tv_nsec = 10000;

  while (!this->stop)
  {
    if (this->IsPaused() && !this->stepInc)
      this->pauseTime += step;
    else
    {
      this->simTime += step;
      this->Update();
    }

    //nanosleep(&req, &rem);

    // TODO: Fix timeout:  this belongs in simulator.cc
    /*if (this->timeout > 0 && this->GetRealTime() > this->timeout)
    {
      this->stop = true;
      break;
    }*/

    if (this->IsPaused() && this->stepInc)
      this->stepInc = false;

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
  event::Events::worldUpdateStartSignal();

  /// Send statistics about the world simulation
  if (common::Time::GetWallTime() - this->prevStatTime > this->statPeriod)
  {
    common::Message::Stamp( this->worldStatsMsg.mutable_header() );
    common::Message::Set( this->worldStatsMsg.mutable_sim_time(), 
        this->GetSimTime());
    common::Message::Set( this->worldStatsMsg.mutable_real_time(),
        this->GetRealTime() );
    common::Message::Set( this->worldStatsMsg.mutable_pause_time(),
        this->GetPauseTime());

    this->statPub->Publish( this->worldStatsMsg );
    this->prevStatTime = common::Time::GetWallTime();
  }

  // Update all the models
  (*this.*modelUpdateFunc)();

  // Update the physics engine
  if (this->physicsEngine)
    this->physicsEngine->UpdatePhysics();

  // TODO: put back in
  //Logger::Instance()->Update();

  event::Events::worldUpdateEndSignal();
}

////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
{
  // Clear out the entity tree
  Model_V::iterator iter;
  for (iter = this->models.begin(); iter != this->models.end(); iter++)
  {
    if (*iter)
    {
      (*iter)->Fini();
    }
  }
  this->models.clear();

  if (this->physicsEngine)
  {
    this->physicsEngine->Fini();
    this->physicsEngine.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Remove all entities from the world
void World::Clear()
{
  // TODO: Implement this
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
common::Param *World::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr << "World::GetParam - Invalid param index\n";

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Return the physics engine
PhysicsEnginePtr World::GetPhysicsEngine() const
{
  return this->physicsEngine;
}

////////////////////////////////////////////////////////////////////////////////
/// Delete an entity by name
void World::DeleteEntityCB(const std::string &name)
{
  // TODO: Implement this function
}

////////////////////////////////////////////////////////////////////////////////
// Get an element by name
BasePtr World::GetByName(const std::string &name)
{
  return this->rootElement->GetByName(name);
}

///////////////////////////////////////////////////////////////////////////////
// Load a model
void World::LoadEntities( common::XMLConfigNode *node, BasePtr parent )
{
  common::XMLConfigNode *cnode;

  cnode = node->GetChild("model");
  while (cnode)
  {
    ModelPtr model( new Model(parent) );
    model->SetWorld(shared_from_this());
    model->Load(cnode);

    // TODO : Put back in the ability to nest models. We should do this
    // without requiring a joint.

    event::Events::addEntitySignal(model->GetCompleteScopedName());
    this->models.push_back(model);

    cnode = cnode->GetNext("model");
  }

  /*if (node)
  {
    // Check for model nodes
    if (node->GetName() == "model")
    {
    }
    // TODO: this shouldn't exist in the physics sim
    else if (node->GetName() == "light")
    {
      msgs::Light msg = common::Message::LightFromXML(node);
      msg.mutable_header()->set_str_id( "light" );
      msg.set_action(msgs::Light::UPDATE);
      this->lightPub->Publish(msg);
    }

    // Load children
    for (cnode = node->GetChild(); cnode != NULL; cnode = cnode->GetNext())
      this->LoadEntities( cnode, parent);
  }*/
}

///////////////////////////////////////////////////////////////////////////////
/// Get the number of models
unsigned int World::GetModelCount() const
{
  return this->models.size();
}

///////////////////////////////////////////////////////////////////////////////
/// Get a model based on an index
ModelPtr World::GetModel(unsigned int index)
{
  ModelPtr model;

  if (index < this->models.size())
    model = this->models[index];
  else
    gzerr << "Invalid model index\n";

  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Reset the simulation to the initial settings
void World::Reset()
{
  Model_V::iterator miter;

  for (miter = this->models.begin(); miter != this->models.end(); miter++)
    (*miter)->Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Pause callback
void World::PauseCB(bool p)
{
}

////////////////////////////////////////////////////////////////////////////////
// Step callback
void World::StepCB()
{
  this->stepInc = true;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void World::SetSelectedEntityCB( const std::string &name )
{
  msgs::Selection msg;
  BasePtr base = this->GetByName(name);
  EntityPtr ent = boost::shared_dynamic_cast<Entity>(base);

  // unselect selectedEntity
  if (this->selectedEntity)
  {
    msg.mutable_header()->set_str_id( this->selectedEntity->GetCompleteScopedName() );
    msg.set_selected( false );
    this->selectionPub->Publish(msg);

    this->selectedEntity->SetSelected(false);
  }

  // if a different entity is selected, show bounding box and SetSelected(true)
  if (ent && this->selectedEntity != ent)
  {
    // set selected entity to ent
    this->selectedEntity = ent;
    this->selectedEntity->SetSelected(true);

    msg.mutable_header()->set_str_id( this->selectedEntity->GetCompleteScopedName() );
    msg.set_selected( true );
    this->selectionPub->Publish(msg);
  }
  else
    this->selectedEntity.reset();
  //event::Events::entitySelectedSignal(this->selectedEntity);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the selected entity
EntityPtr World::GetSelectedEntity() const
{
  return this->selectedEntity;
}

////////////////////////////////////////////////////////////////////////////////
/// Print entity tree
void World::PrintEntityTree()
{
  for (Model_V::iterator iter = this->models.begin();
       iter != this->models.end(); iter++)
  {
    (*iter)->Print("");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulation time
gazebo::common::Time World::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the sim time
void World::SetSimTime(common::Time t)
{
  this->simTime = t;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
gazebo::common::Time World::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
gazebo::common::Time World::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
common::Time World::GetRealTime() const
{
  return common::Time::GetWallTime() - this->startTime;
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

  event::Events::pauseSignal(p);
  this->pause = p;
}

void World::PublishScene( const boost::shared_ptr<msgs::Request const> &data )
{
  msgs::Scene scene;
  common::Message::Init(scene,"scene");

  std::list< boost::shared_ptr<msgs::Visual const> >::iterator iter;
  for (iter = this->visualMsgs.begin(); iter != this->visualMsgs.end(); iter++)
  {
    msgs::Visual *visMsg = scene.add_visual();
    visMsg->CopyFrom( *(*iter) );
  }

  this->BuildSceneMsg(scene, this->rootElement);
  this->scenePub->Publish( scene );
}

////////////////////////////////////////////////////////////////////////////////
// Construct a scene message
void World::BuildSceneMsg(msgs::Scene &scene, BasePtr entity)
{
  if (entity->HasType(Entity::ENTITY))
  {
    msgs::Pose *poseMsg = scene.add_pose();
    common::Pose3d pose = boost::shared_static_cast<Entity>(entity)->GetRelativePose();
    poseMsg->CopyFrom( common::Message::Convert(pose) );
    common::Message::Init(*poseMsg, entity->GetCompleteScopedName() );
  }

  for (unsigned int i=0; i < entity->GetChildCount(); i++)
  {
    this->BuildSceneMsg( scene, entity->GetChild(i) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Log the visuals, which allows the world to maintain the current state of
// the scene. This in turns allows a gui to query the latest state.
void World::VisualLog(const boost::shared_ptr<msgs::Visual const> &msg)
{
  std::list< boost::shared_ptr<msgs::Visual const> >::iterator iter;
  std::list< boost::shared_ptr<msgs::Visual const> >::iterator prev;

  iter = this->visualMsgs.begin();
  while (iter != this->visualMsgs.end())
  {
    if ( (*iter)->header().str_id() == msg->header().str_id() )
    {
      prev = iter++;
      this->visualMsgs.erase(prev);
    }
    else
      iter++;
  }

  this->visualMsgs.push_back( msg );
}

////////////////////////////////////////////////////////////////////////////////
// TBB version of model updating
void World::ModelUpdateTBB()
{
  tbb::parallel_for( tbb::blocked_range<size_t>(0, this->models.size(), 10),
      ModelUpdate_TBB(&this->models) );
}

////////////////////////////////////////////////////////////////////////////////
// Single loop verison of model updating
void World::ModelUpdateSingleLoop()
{
  for (Model_V::iterator iter = this->models.begin(); 
       iter != this->models.end(); iter++)
  {
    (*iter)->Update();
  }
}
