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
#include <boost/thread/mutex.hpp>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "transport/Node.hh"
#include "transport/Transport.hh"
#include "transport/Publisher.hh"
#include "transport/Subscriber.hh"

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
// constructor
World::World(const std::string &name)
{
  this->stepInc = false;
  this->pause = false;

  this->name = name;

  this->updateMutex = new boost::mutex();

  common::Param::Begin(&this->parameters);
  // NO Parameters...
  common::Param::End();


  this->connections.push_back( 
     event::Events::ConnectStepSignal( boost::bind(&World::OnStep, this) ) );
  this->connections.push_back( 
     event::Events::ConnectSetSelectedEntitySignal( boost::bind(&World::SetSelectedEntityCB, this, _1) ) );
  this->connections.push_back(
     event::Events::ConnectDeleteEntitySignal( boost::bind(&World::DeleteEntityCB, this, _1) ) );
}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  this->connections.clear();
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load(common::XMLConfigNode *rootNode)//, unsigned int serverId)
{
  common::XMLConfigNode *physicsNode = NULL;

  // DO THIS FIRST
  if (rootNode)
  {
    this->sceneMsg.CopyFrom( 
        common::Message::SceneFromXML(rootNode->GetChild("scene")) );
    physicsNode = rootNode->GetChild("physics");
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  // The period at which statistics about the world are published
  this->statPeriod = common::Time(0,200000000);


  common::Message::Init( this->worldStatsMsg, "statistics" );

  this->factorySub = this->node->Subscribe("~/factory", 
                                           &World::OnFactoryMsg, this);


  this->controlSub = this->node->Subscribe("~/world_control", 
                                           &World::OnControl, this);

  this->sceneSub = this->node->Subscribe("~/publish_scene", 
                                        &World::PublishScene, this);
  this->visSub = this->node->Subscribe("~/visual", &World::VisualLog, this);

  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");
  this->statPub = this->node->Advertise<msgs::WorldStatistics>("~/world_stats");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");
  this->newEntityPub = this->node->Advertise<msgs::Entity>("~/new_entity");

  if (physicsNode)
  {
    std::string type = physicsNode->GetString("type","ode",1);

    this->physicsEngine = PhysicsFactory::NewPhysicsEngine( type, shared_from_this());
  }
  else
  {
    this->physicsEngine = PhysicsFactory::NewPhysicsEngine("ode", shared_from_this());
  }

  if (this->physicsEngine == NULL)
    gzthrow("Unable to create physics engine\n");

  // This should come before loading of entities
  this->physicsEngine->Load(physicsNode);

  this->rootElement.reset(new Base(BasePtr()));
  this->rootElement->SetName("root");
  this->rootElement->SetWorld(shared_from_this());

  // Create all the entities
  if (rootNode)
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

  stream << prefix << "  " << this->name;

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
  struct timespec req;//, rem;

  this->startTime = common::Time::GetWallTime();

  // Set a default sleep time
  req.tv_sec  = 0;
  req.tv_nsec = 10000;

  while (!this->stop)
  {
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


    if (this->IsPaused() && !this->stepInc)
      this->pauseTime += step;
    else
    {
      boost::mutex::scoped_lock lock(*this->updateMutex);
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
  return this->name;
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

////////////////////////////////////////////////////////////////////////////////
// Load a model 
ModelPtr World::LoadModel(common::XMLConfigNode *node, BasePtr parent)
{
  ModelPtr model( new Model(parent) );
  model->SetWorld(shared_from_this());
  model->Load(node);

  event::Events::addEntitySignal(model->GetCompleteScopedName());
  this->models.push_back(model);

  msgs::Entity msg;
  common::Message::Init(msg, model->GetCompleteScopedName() );
  msg.set_name(model->GetCompleteScopedName());

  this->newEntityPub->Publish(msg);
  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Load a model
void World::LoadEntities( common::XMLConfigNode *node, BasePtr parent )
{
  common::XMLConfigNode *cnode;

  cnode = node->GetChild("model");
  while (cnode)
  {
    this->LoadModel(cnode, parent);

    // TODO : Put back in the ability to nest models. We should do this
    // without requiring a joint.

    cnode = cnode->GetNext("model");
  }

  cnode = node->GetChild("light");
  while (cnode)
  {
    msgs::Light *lm = this->sceneMsg.add_light();
    lm->CopyFrom( common::Message::LightFromXML(cnode) );

    cnode = cnode->GetNext("light");
  }
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
// Step callback
void World::OnStep()
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
  if (this->pause)
    return (this->pauseStartTime - this->startTime) - this->realTimeOffset;
  else
    return (common::Time::GetWallTime() - this->startTime) - this->realTimeOffset;
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

  if (p)
    this->pauseStartTime = common::Time::GetWallTime();
  else
    this->realTimeOffset += common::Time::GetWallTime() - this->pauseStartTime;

  event::Events::pauseSignal(p);
  this->pause = p;
}

void World::OnControl( const boost::shared_ptr<msgs::WorldControl const> &data )
{
  if (data->has_pause())
    this->SetPaused( data->pause() );

  if (data->has_step())
    this->OnStep();
}

void World::PublishScene( const boost::shared_ptr<msgs::Request const> &data )
{
  common::Message::Stamp(this->sceneMsg.mutable_header());

  this->sceneMsg.clear_pose();
  if (this->rootElement)
  {
    this->BuildSceneMsg(this->sceneMsg, this->rootElement);
    this->scenePub->Publish( this->sceneMsg );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Construct a scene message
void World::BuildSceneMsg(msgs::Scene &scene, BasePtr entity)
{
  if (entity)
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
}

////////////////////////////////////////////////////////////////////////////////
// Log the visuals, which allows the world to maintain the current state of
// the scene. This in turns allows a gui to query the latest state.
void World::VisualLog(const boost::shared_ptr<msgs::Visual const> &msg)
{
  int i = 0;
  for (; i < this->sceneMsg.visual_size(); i++)
  {
    if (this->sceneMsg.visual(i).header().str_id() == msg->header().str_id())
    {
      this->sceneMsg.mutable_visual(i)->CopyFrom(*msg);
      break;
    }
  }

  if (i >= this->sceneMsg.visual_size())
  {
    msgs::Visual *newVis = this->sceneMsg.add_visual();
    newVis->CopyFrom(*msg);
  }
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

////////////////////////////////////////////////////////////////////////////////
// Received a factory msg
void World::OnFactoryMsg( const boost::shared_ptr<msgs::Factory const> &msg)
{
  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();
 
  try
  {
    xmlFile->LoadString(msg->xml());
  }
  catch (common::Exception e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());

  {
    boost::mutex::scoped_lock lock(*this->updateMutex);
    // Add the new models into the World
    ModelPtr model = this->LoadModel( rootNode, BasePtr() );
    model->Init();
  }
}
