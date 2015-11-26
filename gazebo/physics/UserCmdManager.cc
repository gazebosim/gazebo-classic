/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/range/adaptor/reversed.hpp>

#include "gazebo/common/Events.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldState.hh"

#include "gazebo/physics/UserCmdManagerPrivate.hh"
#include "gazebo/physics/UserCmdManager.hh"

using namespace gazebo;
using namespace physics;


/////////////////////////////////////////////////
UserCmd::UserCmd(UserCmdManagerPtr _manager,
                 const unsigned int _id,
                 physics::WorldPtr _world,
                 const std::string &_description,
                 const msgs::UserCmd::Type &_type)
  : dataPtr(new UserCmdPrivate())
{
  this->dataPtr->manager = _manager;
  this->dataPtr->id = _id;
  this->dataPtr->world = _world;
  this->dataPtr->description = _description;
  this->dataPtr->type = _type;
  this->dataPtr->sdf = NULL;

  // Record current world state
  this->dataPtr->startState = WorldState(this->dataPtr->world);
}

/////////////////////////////////////////////////
UserCmd::~UserCmd()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void UserCmd::Undo()
{
  // Record / override the state for redo
  this->dataPtr->endState = WorldState(this->dataPtr->world);

  // Undo insertion
  if (this->dataPtr->type == msgs::UserCmd::INSERTING &&
      this->dataPtr->manager && !this->dataPtr->entityName.empty())
  {
    // Keep sdf for redo
    if (!this->dataPtr->sdf)
    {
      physics::ModelPtr model =
          this->dataPtr->world->GetModel(this->dataPtr->entityName);
      physics::LightPtr light =
          this->dataPtr->world->Light(this->dataPtr->entityName);
      if (!model && !light)
      {
        gzerr << "Entity [" << this->dataPtr->entityName << "] not found."
            << std::endl;
        return;
      }

      this->dataPtr->sdf.reset(new sdf::SDF);
      if (model)
        this->dataPtr->sdf->Root()->Copy(model->GetSDF());
      else if (light)
        this->dataPtr->sdf->Root()->Copy(light->GetSDF());
    }

    // Delete
    transport::requestNoReply(this->dataPtr->manager->dataPtr->node,
        "entity_delete", this->dataPtr->entityName);
  }

  // Set state to the moment the command was executed
  this->dataPtr->manager->dataPtr->pendingStates.push_back(
      this->dataPtr->startState);
}

/////////////////////////////////////////////////
void UserCmd::Redo()
{
  // Redo insertion
  if (this->dataPtr->type == msgs::UserCmd::INSERTING &&
      this->dataPtr->manager && this->dataPtr->sdf)
  {
    msgs::Factory msg;
    msg.set_sdf(this->dataPtr->sdf->ToString());
    this->dataPtr->manager->dataPtr->modelFactoryPub->Publish(msg);

    this->dataPtr->manager->dataPtr->insertionPending =
        this->dataPtr->entityName;
  }

  // Set state to the moment undo was triggered
  this->dataPtr->manager->dataPtr->pendingStates.push_back(
      this->dataPtr->endState);
}

/////////////////////////////////////////////////
unsigned int UserCmd::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
std::string UserCmd::Description() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
msgs::UserCmd::Type UserCmd::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void UserCmd::SetEntityName(const std::string &_name)
{
  this->dataPtr->entityName = _name;
}

/////////////////////////////////////////////////
std::string UserCmd::EntityName() const
{
  return this->dataPtr->entityName;
}

/////////////////////////////////////////////////
UserCmdManager::UserCmdManager(const WorldPtr _world)
  : dataPtr(new UserCmdManagerPrivate())
{
  this->dataPtr->world = _world;

  // Events
  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&UserCmdManager::ProcessPendingStates, this)));

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->userCmdSub = this->dataPtr->node->Subscribe("~/user_cmd",
      &UserCmdManager::OnUserCmdMsg, this, true);

  this->dataPtr->undoRedoSub = this->dataPtr->node->Subscribe("~/undo_redo",
      &UserCmdManager::OnUndoRedoMsg, this);

  this->dataPtr->userCmdStatsPub =
    this->dataPtr->node->Advertise<msgs::UserCmdStats>("~/user_cmd_stats");

  this->dataPtr->modelModifyPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->lightModifyPub =
      this->dataPtr->node->Advertise<msgs::Light>("~/light/modify");

  this->dataPtr->modelFactoryPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");

  this->dataPtr->lightFactoryPub =
      this->dataPtr->node->Advertise<msgs::Light>("~/factory/light");

  this->dataPtr->idCounter = 0;
  this->dataPtr->insertionPending = "";
}

/////////////////////////////////////////////////
UserCmdManager::~UserCmdManager()
{
  this->dataPtr->connections.clear();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void UserCmdManager::OnUserCmdMsg(ConstUserCmdPtr &_msg)
{
  // Generate unique id
  unsigned int id = this->dataPtr->idCounter++;

  // Create command
  UserCmdPtr cmd(new UserCmd(shared_from_this(), id, this->dataPtr->world,
      _msg->description(), _msg->type()));

  if (_msg->has_entity_name())
    cmd->SetEntityName(_msg->entity_name());

  // Forward command now that we saved the previous world state
  if (_msg->type() == msgs::UserCmd::MOVING)
  {
    for (int i = 0; i < _msg->model_size(); ++i)
      this->dataPtr->modelModifyPub->Publish(_msg->model(i));

    for (int i = 0; i < _msg->light_size(); ++i)
      this->dataPtr->lightModifyPub->Publish(_msg->light(i));
  }
  else if (_msg->type() == msgs::UserCmd::INSERTING)
  {
    if (_msg->has_factory())
    {
      this->dataPtr->modelFactoryPub->Publish(_msg->factory());
    }
    else if (_msg->light_size() == 1)
    {
      this->dataPtr->lightFactoryPub->Publish(_msg->light(0));
    }
    else
    {
      gzwarn << "Insert command [" << _msg->description() <<
          "] does not contain factory or light messages." <<
          " Command won't be executed." << std::endl;
    }
  }

  // Add it to undo list
  this->dataPtr->undoCmds.push_back(cmd);

  // Clear redo list
  this->dataPtr->redoCmds.clear();

  // Publish stats
  this->PublishCurrentStats();
}

/////////////////////////////////////////////////
void UserCmdManager::OnUndoRedoMsg(ConstUndoRedoPtr &_msg)
{
  // Undo
  if (_msg->undo())
  {
    if (this->dataPtr->undoCmds.empty())
    {
      gzwarn << "No commands to be undone" << std::endl;
      return;
    }

    // Get the last done command
    UserCmdPtr cmd = this->dataPtr->undoCmds.back();

    // If there's an id, get that command instead
    if (_msg->has_id())
    {
      bool found = false;
      for (auto cmdIt : this->dataPtr->undoCmds)
      {
        if (cmdIt->Id() == _msg->id())
        {
          cmd = cmdIt;
          found = true;
          break;
        }
      }
      if (!found)
      {
        gzerr << "Requested command [" << _msg->id() <<
            "] is not in the undo queue and won't be undone." << std::endl;
        return;
      }
    }

    // Undo all commands up to the desired one
    for (auto cmdIt : boost::adaptors::reverse(this->dataPtr->undoCmds))
    {
      // Undo it
      cmdIt->Undo();

      // Transfer to the redo list
      this->dataPtr->undoCmds.pop_back();
      this->dataPtr->redoCmds.push_back(cmdIt);

      if (cmdIt == cmd)
        break;
    }
  }
  // Redo
  else
  {
    if (this->dataPtr->redoCmds.empty())
    {
      gzwarn << "No commands to be undone" << std::endl;
      return;
    }

    // Get last undone command
    UserCmdPtr cmd = this->dataPtr->redoCmds.back();

    // If there's an id, get that command instead
    if (_msg->has_id())
    {
      bool found = false;
      for (auto cmdIt : this->dataPtr->redoCmds)
      {
        if (cmdIt->Id() == _msg->id())
        {
          cmd = cmdIt;
          found = true;
          break;
        }
      }
      if (!found)
      {
        gzerr << "Requested command [" << _msg->id() <<
            "] is not in the redo queue and won't be redone." << std::endl;
        return;
      }
    }

    // Redo all commands up to the desired one
    for (auto cmdIt : boost::adaptors::reverse(this->dataPtr->redoCmds))
    {
      // Redo it
      cmdIt->Redo();

      // Transfer to the undo list
      this->dataPtr->redoCmds.pop_back();
      this->dataPtr->undoCmds.push_back(cmdIt);

      if (cmdIt == cmd)
        break;
    }
  }

  this->PublishCurrentStats();
}

/////////////////////////////////////////////////
void UserCmdManager::PublishCurrentStats()
{
  msgs::UserCmdStats statsMsg;

  statsMsg.set_undo_cmd_count(this->dataPtr->undoCmds.size());
  statsMsg.set_redo_cmd_count(this->dataPtr->redoCmds.size());

  for (auto cmd : this->dataPtr->undoCmds)
  {
    msgs::UserCmd *msg = statsMsg.add_undo_cmd();
    msg->set_id(cmd->Id());
    msg->set_description(cmd->Description());
    msg->set_type(cmd->Type());
  }

  for (auto cmd : this->dataPtr->redoCmds)
  {
    msgs::UserCmd *msg = statsMsg.add_redo_cmd();
    msg->set_id(cmd->Id());
    msg->set_description(cmd->Description());
    msg->set_type(cmd->Type());
  }

  this->dataPtr->userCmdStatsPub->Publish(statsMsg);
}

/////////////////////////////////////////////////
void UserCmdManager::ProcessPendingStates()
{
  if (this->dataPtr->pendingStates.empty())
    return;

  // Insertion pending
  if (!this->dataPtr->insertionPending.empty())
  {
    // Model hasn't been inserted yet
    if (!this->dataPtr->world->GetModel(this->dataPtr->insertionPending))
      return;
    else
      this->dataPtr->insertionPending = "";
  }

  // Reset physics states for the whole world
  this->dataPtr->world->ResetPhysicsStates();

  // Set world state to pending state
  this->dataPtr->world->SetState(this->dataPtr->pendingStates.front());
  this->dataPtr->pendingStates.pop_front();
}

