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
#include <boost/thread/recursive_mutex.hpp>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "sdf/parser/parser.hh"
#include "transport/Node.hh"
#include "transport/Transport.hh"
#include "transport/Publisher.hh"
#include "transport/Subscriber.hh"

#include "common/Diagnostics.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Plugin.hh"

#include "physics/Link.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Model.hh"
#include "physics/World.hh"

#include "sensors/Sensor.hh"
#include "sensors/CameraSensor.hh"

#include "physics/Collision.hh"

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
World::World(const std::string &_name)
{
  this->receiveMutex = new boost::mutex();

  this->stepInc = false;
  this->pause = false;
  this->thread = NULL;

  this->name = _name;

  this->updateMutex = new boost::mutex();
  this->incomingMsgMutex = new boost::recursive_mutex();
  this->modelWorldPoseUpdateMutex = new boost::recursive_mutex();

  this->connections.push_back( 
     event::Events::ConnectStep( boost::bind(&World::OnStep, this) ) );
  this->connections.push_back( 
     event::Events::ConnectSetSelectedEntity( boost::bind(&World::SetSelectedEntityCB, this, _1) ) );
  this->connections.push_back(
     event::Events::ConnectDeleteEntity( boost::bind(&World::DeleteEntityCB, this, _1) ) );

}

////////////////////////////////////////////////////////////////////////////////
// Private destructor
World::~World()
{
  delete this->updateMutex;
  this->updateMutex = NULL;

  delete this->incomingMsgMutex;
  this->incomingMsgMutex = NULL;


  delete this->modelWorldPoseUpdateMutex;
  this->modelWorldPoseUpdateMutex = NULL;

  this->connections.clear();
  this->Fini();

  this->sdf->Reset();
  this->rootElement.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the world
void World::Load( sdf::ElementPtr _sdf )
{
  this->sdf = _sdf;

  if (this->sdf->GetValueString("name").empty())
    gzwarn << "create_world(world_name=[" 
           << this->name << "]) overwrites sdf world name\n!";
  else
    this->name = this->sdf->GetValueString("name");

  this->sceneMsg.CopyFrom( 
        msgs::SceneFromSDF(_sdf->GetElement("scene")) );

  // The period at which statistics about the world are published
  this->statPeriod = common::Time(0,200000000);


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->GetName());

  this->guiPub = this->node->Advertise<msgs::GUI>("~/gui");

  if (this->sdf->HasElement("gui"))
    this->guiPub->Publish(msgs::GUIFromSDF(this->sdf->GetElement("gui")));

  this->factorySub = this->node->Subscribe("~/factory", 
                                           &World::OnFactoryMsg, this);

  this->controlSub = this->node->Subscribe("~/world_control", 
                                           &World::OnControl, this);

  this->requestSub = this->node->Subscribe("~/request",&World::OnRequest, this);
  this->responsePub = this->node->Advertise<msgs::Response>("~/response");
  this->sceneSub = this->node->Subscribe("~/scene", 
                                        &World::OnScene, this);

  this->visSub = this->node->Subscribe("~/visual", &World::VisualLog, this);
  this->jointSub = this->node->Subscribe("~/joint", &World::JointLog, this);

  this->statPub = this->node->Advertise<msgs::WorldStatistics>("~/world_stats");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/info");

  this->modelSub = this->node->Subscribe<msgs::Model>("~/model/modify",
      &World::OnModelMsg, this);


  std::string type = _sdf->GetElement("physics")->GetValueString("type");
  this->physicsEngine = PhysicsFactory::NewPhysicsEngine( type, 
                                                          shared_from_this());

  if (this->physicsEngine == NULL)
    gzthrow("Unable to create physics engine\n");

  // This should come before loading of entities
  this->physicsEngine->Load( _sdf->GetElement("physics") );

  this->rootElement.reset(new Base(BasePtr()));
  this->rootElement->SetName(this->GetName());
  WorldPtr me = shared_from_this();
  this->rootElement->SetWorld(me);

  // Create all the entities
  this->LoadEntities(_sdf, this->rootElement);

  // TODO: Performance test to see if TBB model updating is necessary
  // Choose threaded or unthreaded model updating depending on the number of
  // models in the scene
  //if (this->GetModelCount() < 20)
    this->modelUpdateFunc = &World::ModelUpdateSingleLoop;
  //else
    //this->modelUpdateFunc = &World::ModelUpdateTBB;

  event::Events::worldCreated(this->GetName());
}


////////////////////////////////////////////////////////////////////////////////
// Initialize the world
void World::Init()
{
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Init();

  // Initialize the physics engine
  this->physicsEngine->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Run the world in a thread
void World::Run()
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

  this->startTime = common::Time::GetWallTime();

  // This fixes a minor issue when the world is paused before it's started
  if (this->IsPaused())
    this->pauseStartTime = this->startTime;

  while (!this->stop)
  {
    /// Send statistics about the world simulation
    if (common::Time::GetWallTime() - this->prevStatTime > this->statPeriod)
    {
      msgs::Set( this->worldStatsMsg.mutable_sim_time(), this->GetSimTime());
      msgs::Set( this->worldStatsMsg.mutable_real_time(), this->GetRealTime() );
      msgs::Set( this->worldStatsMsg.mutable_pause_time(),this->GetPauseTime());
      this->worldStatsMsg.set_paused( this->IsPaused() ); 

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

    // TODO: Fix timeout:  this belongs in simulator.cc
    /*if (this->timeout > 0 && this->GetRealTime() > this->timeout)
    {
      this->stop = true;
      break;
    }*/

    if (this->IsPaused() && this->stepInc)
      this->stepInc = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the world
void World::Update()
{
  event::Events::worldUpdateStart();

  // Update all the models
  (*this.*modelUpdateFunc)();

  // TODO: put back in
  //Logger::Instance()->Update();

  // Update the physics engine
  if (this->physicsEngine)
  {
    this->physicsEngine->UpdateCollision();

    for (std::list<Entity*>::iterator iter = this->dirtyPoses.begin(); 
        iter != this->dirtyPoses.end(); iter++)
    {
      (*iter)->SetWorldPose( (*iter)->GetDirtyPose(), false );
    }
    this->dirtyPoses.clear();

    this->physicsEngine->UpdatePhysics();
  }

  this->ProcessEntityMsgs();

  event::Events::worldUpdateEnd();
}

////////////////////////////////////////////////////////////////////////////////
void World::ProcessEntityMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  std::list< std::string >::iterator iter;
  for (iter = this->deleteEntity.begin(); 
       iter != this->deleteEntity.end(); iter++)
  {
    this->rootElement->RemoveChild( (*iter) );
  }

  this->deleteEntity.clear();
}


////////////////////////////////////////////////////////////////////////////////
// Finilize the world
void World::Fini()
{
  this->Stop();
  this->plugins.clear();

  this->rootElement->Fini();

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
void World::DeleteEntityCB(const std::string &/*_name*/)
{
  // TODO: Implement this function
}

////////////////////////////////////////////////////////////////////////////////
// Get an element by name
BasePtr World::GetByName(const std::string &_name)
{
  return this->rootElement->GetByName(_name);
}

////////////////////////////////////////////////////////////////////////////////
/// Get a model by name
ModelPtr World::GetModelByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Model>( this->GetByName( _name ) );
}

////////////////////////////////////////////////////////////////////////////////
// Get a pointer to a model based on a name
EntityPtr World::GetEntityByName(const std::string &_name)
{
  return boost::shared_dynamic_cast<Entity>( this->GetByName( _name ) );
}


////////////////////////////////////////////////////////////////////////////////
// Load a model 
ModelPtr World::LoadModel( sdf::ElementPtr &_sdf , BasePtr _parent)
{
  ModelPtr model( new Model(_parent) );
  model->SetWorld(shared_from_this());
  model->Load(_sdf);

  event::Events::addEntity(model->GetCompleteScopedName());

  msgs::Model msg;
  model->FillModelMsg(msg);
  this->modelPub->Publish(msg);

  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Load a model
void World::LoadEntities( sdf::ElementPtr &_sdf, BasePtr _parent )
{
  if (_sdf->HasElement("light"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("light");
    while (childElem)
    {
      msgs::Light *lm = this->sceneMsg.add_light();
      lm->CopyFrom( msgs::LightFromSDF(childElem) );

      childElem = _sdf->GetNextElement("light", childElem);
    }
  }

  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr childElem = _sdf->GetElement("model");

    while (childElem)
    {
      this->LoadModel(childElem, _parent);

      // TODO : Put back in the ability to nest models. We should do this
      // without requiring a joint.

      childElem = _sdf->GetNextElement("model", childElem);
    }
  }

  // Load the plugins
  if (_sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = _sdf->GetNextElement("plugin", pluginElem);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
/// Get the number of models
unsigned int World::GetModelCount() const
{
  return this->rootElement->GetChildCount();
}

///////////////////////////////////////////////////////////////////////////////
/// Get a model based on an index
ModelPtr World::GetModel(unsigned int _index)
{
  ModelPtr model;

  if (_index < this->rootElement->GetChildCount() &&
      this->rootElement->GetChild(_index)->HasType(Base::MODEL))
    model = boost::shared_static_cast<Model>(this->rootElement->GetChild(_index));
  else
    gzerr << "Invalid model index\n";

  return model;
}

///////////////////////////////////////////////////////////////////////////////
// Reset the simulation to the initial settings
void World::Reset()
{
  // Reset all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Reset();
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
    msg.set_name( this->selectedEntity->GetCompleteScopedName() );
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

    msg.set_name( this->selectedEntity->GetCompleteScopedName() );
    msg.set_selected( true );

    this->selectionPub->Publish(msg);
  }
  else
    this->selectedEntity.reset();
  //event::Events::entitySelected(this->selectedEntity);
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
  // Initialize all the entities
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Print("");
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

  event::Events::pause(p);
  this->pause = p;
}

void World::OnControl( const boost::shared_ptr<msgs::WorldControl const> &data )
{
  if (data->has_pause())
    this->SetPaused( data->pause() );

  if (data->has_step())
    this->OnStep();
}

void World::OnRequest( const boost::shared_ptr<msgs::Request const> &_msg )
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  msgs::Response response;
  response.set_id( _msg->id() );
  response.set_request( _msg->request() );
  response.set_response( "success" );

  if (_msg->request() == "entity_list")
  {
    msgs::Model_V modelVMsg;

    for (unsigned int i=0; i < this->rootElement->GetChildCount(); i++)
    {
      BasePtr entity = this->rootElement->GetChild(i);
      msgs::Model *modelMsg = modelVMsg.add_models();
      if (entity->HasType(Base::MODEL))
      {
        ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
        model->FillModelMsg( *modelMsg );
      }
    }

    response.set_type( modelVMsg.GetTypeName() );
    std::string *serializedData = response.mutable_serialized_data();
    modelVMsg.SerializeToString( serializedData );
  }
  else if (_msg->request() == "entity_delete")
  {
    this->deleteEntity.push_back( _msg->data() );
  }
  else if (_msg->request() == "entity_info")
  {
    BasePtr entity = this->rootElement->GetByName( _msg->data() );
    if (entity)
    {
      if (entity->HasType(Base::MODEL))
      {
        msgs::Model modelMsg;
        ModelPtr model = boost::shared_dynamic_cast<Model>(entity);
        model->FillModelMsg(modelMsg);

        std::string *serializedData = response.mutable_serialized_data();
        modelMsg.SerializeToString( serializedData );
        response.set_type( modelMsg.GetTypeName() );
      }
      else if (entity->HasType(Base::LINK))
      {
        msgs::Link linkMsg;
        LinkPtr link = boost::shared_dynamic_cast<Link>(entity);
        link->FillLinkMsg(linkMsg);

        std::string *serializedData = response.mutable_serialized_data();
        linkMsg.SerializeToString( serializedData );
        response.set_type( linkMsg.GetTypeName() );
      }
      else if (entity->HasType(Base::COLLISION))
      {
        msgs::Collision collisionMsg;
        CollisionPtr collision = boost::shared_dynamic_cast<Collision>(entity);
        collision->FillCollisionMsg( collisionMsg );

        std::string *serializedData = response.mutable_serialized_data();
        collisionMsg.SerializeToString( serializedData );
        response.set_type( collisionMsg.GetTypeName() );
      }
      else if (entity->HasType(Base::JOINT))
      {
        msgs::Joint jointMsg;
        JointPtr joint = boost::shared_dynamic_cast<Joint>(entity);
        joint->FillJointMsg( jointMsg );

        std::string *serializedData = response.mutable_serialized_data();
        jointMsg.SerializeToString( serializedData );
        response.set_type( jointMsg.GetTypeName() );
      }
    }
    else
    {
      response.set_type("error");
      response.set_response( "nonexistant" );
    }
  }
  else if (_msg->request() == "scene_info")
  {
    this->sceneMsg.clear_model();
    this->BuildSceneMsg( this->sceneMsg, this->rootElement );

    std::string *serializedData = response.mutable_serialized_data();
    this->sceneMsg.SerializeToString( serializedData );
    response.set_type( sceneMsg.GetTypeName() );
    
  }

  this->responsePub->Publish( response );
}

void World::OnScene( const boost::shared_ptr<msgs::Scene const> &_data )
{
  this->incomingMsgMutex->lock();
  this->sceneMsg.MergeFrom( *_data );
  this->incomingMsgMutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Construct a scene message
void World::BuildSceneMsg(msgs::Scene &scene, BasePtr entity)
{
  if (entity)
  {
    if (entity->HasType(Entity::MODEL))
    {
      msgs::Model *modelMsg = scene.add_model();
      this->BuildModelMsg(modelMsg, boost::shared_static_cast<Model>(entity));
    }

    for (unsigned int i=0; i < entity->GetChildCount(); i++)
    {
      this->BuildSceneMsg( scene, entity->GetChild(i) );
    }
  }
}

void World::BuildModelMsg(msgs::Model *_msg, ModelPtr _model)
{
  math::Pose pose = _model->GetWorldPose();
  _msg->set_name( _model->GetCompleteScopedName() );
  msgs::Set(_msg->mutable_pose(), pose);

  for (unsigned int i=0; i < _model->GetChildCount(); i++)
  {
    if (_model->GetChild(i)->HasType(Entity::LINK))
    {
      msgs::Link *linkMsg = _msg->add_links();
      LinkPtr link = boost::shared_static_cast<Link>(_model->GetChild(i));
      this->BuildLinkMsg(linkMsg, link);
    }
  }
}

void World::BuildLinkMsg(msgs::Link *_msg, LinkPtr _link)
{
  math::Pose pose = _link->GetWorldPose();
  _msg->set_name( _link->GetCompleteScopedName() );
  msgs::Set(_msg->mutable_pose(), pose);

  for (unsigned int i=0; i < _link->GetSensorCount(); i++)
  {
    msgs::Sensor *sensorMsg = _msg->add_sensors();
    sensors::SensorPtr sensor = _link->GetSensor(i);
    this->BuildSensorMsg(sensorMsg, sensor);
  }
}

void World::BuildSensorMsg(msgs::Sensor *_msg, sensors::SensorPtr _sensor)
{
  math::Pose pose = _sensor->GetPose();
  _msg->set_name( _sensor->GetName() );
  _msg->set_type( _sensor->GetType() );
  _msg->set_parent(_sensor->GetParentName());
  msgs::Set(_msg->mutable_pose(), pose);

  _msg->set_visualize(_sensor->GetVisualize());
  _msg->set_topic(_sensor->GetTopic());

  if (_sensor->GetType() == "camera")
  {
    sensors::CameraSensorPtr camSensor =
      boost::shared_static_cast<sensors::CameraSensor>(_sensor);
    msgs::CameraSensor *camMsg = _msg->mutable_camera();
    camMsg->mutable_image_size()->set_x(camSensor->GetImageWidth());
    camMsg->mutable_image_size()->set_y(camSensor->GetImageHeight());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Log the visuals, which allows the world to maintain the current state of
// the scene. This in turns allows a gui to query the latest state.
void World::VisualLog(const boost::shared_ptr<msgs::Visual const> &msg)
{
  if (msg->name().find("__GZ_USER_") != std::string::npos)
    return;

  int i = 0;
  for (; i < this->sceneMsg.visual_size(); i++)
  {
    if (this->sceneMsg.visual(i).name() == msg->name())
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
// Log the joint, which allows the world to maintain the current state of
// the scene. This in turns allows a gui to query the latest state.
void World::JointLog(const boost::shared_ptr<msgs::Joint const> &msg)
{
  int i = 0;
  for (; i < this->sceneMsg.joint_size(); i++)
  {
    if (this->sceneMsg.joint(i).name() == msg->name())
    {
      this->sceneMsg.mutable_joint(i)->CopyFrom(*msg);
      break;
    }
  }

  if (i >= this->sceneMsg.joint_size())
  {
    msgs::Joint *newJoint = this->sceneMsg.add_joint();
    newJoint->CopyFrom(*msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
// TBB version of model updating
/*void World::ModelUpdateTBB()
{
  tbb::parallel_for( tbb::blocked_range<size_t>(0, this->models.size(), 10),
      ModelUpdate_TBB(&this->models) );
}*/

////////////////////////////////////////////////////////////////////////////////
// Single loop verison of model updating
void World::ModelUpdateSingleLoop()
{
  // Update all the models
  for (unsigned int i = 0; i < this->rootElement->GetChildCount(); i++)
    this->rootElement->GetChild(i)->Update();
}

////////////////////////////////////////////////////////////////////////////////
// Received a factory msg
void World::OnFactoryMsg( const boost::shared_ptr<msgs::Factory const> &_msg)
{
  sdf::SDFPtr factorySDF(new sdf::SDF);
  sdf::initFile( "/sdf/gazebo.sdf", factorySDF );

  if (_msg->has_sdf())
  {
    // SDF Parsing happens here
    sdf::readString( _msg->sdf(), factorySDF );
  }
  else
    sdf::readFile( _msg->sdf_filename(), factorySDF);

  if (_msg->has_edit_name())
  {
    factorySDF->PrintValues();

    BasePtr base = this->rootElement->GetByName( _msg->edit_name() );
    if (base)
    {
      sdf::ElementPtr elem;
      if (factorySDF->root->GetName() == "gazebo")
        elem = factorySDF->root->GetFirstElement();
      else
        elem = factorySDF->root;

      boost::mutex::scoped_lock lock(*this->updateMutex);
      base->UpdateParameters( elem );
    }
  }
  else
  {
    boost::mutex::scoped_lock lock(*this->updateMutex);
    // Add the new models into the World
    //gzgdb << "factorySDF [" << factorySDF->ToString() << "]\n";
    sdf::ElementPtr elem = factorySDF->root->GetElement("model");
    if (!elem) 
      elem = factorySDF->root->GetElement("world")->GetElement("model");

    ModelPtr model = this->LoadModel( elem, this->rootElement );
    if (_msg->has_pose())
      model->SetWorldPose( msgs::Convert( _msg->pose() ) );

    model->Init();
  }

  factorySDF->root.reset();
}

void World::OnModelMsg( const boost::shared_ptr<msgs::Model const> &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  ModelPtr model = this->GetModelByName( _msg->name() );
  if (!model)
  {
    gzerr << "Unable to find model[" << _msg->name() << "]\n";
    return;
  }

  if (_msg->has_pose())
  {
    model->SetWorldPose( msgs::Convert(_msg->pose()) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load a plugin
void World::LoadPlugin( sdf::ElementPtr &_sdf )
{
  std::string name = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  gazebo::WorldPluginPtr plugin = gazebo::WorldPlugin::Create(filename, name);
  if (plugin)
  {
    WorldPtr myself = shared_from_this();
    plugin->Load(myself,_sdf);
    this->plugins.push_back( plugin );
  }
}

