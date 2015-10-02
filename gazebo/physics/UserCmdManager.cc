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

#include "gazebo/transport/transport.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldState.hh"

#include "gazebo/physics/UserCmdManagerPrivate.hh"
#include "gazebo/physics/UserCmdManager.hh"

using namespace gazebo;
using namespace physics;


/////////////////////////////////////////////////
UserCmd::UserCmd(std::string _id,
                 physics::WorldPtr _world,
                 const std::string &_description,
                 msgs::UserCmd::Type _type,
                 const std::string &_name)
  : dataPtr(new UserCmdPrivate())
{
  this->dataPtr->id = _id;
  this->dataPtr->world = _world;
  this->dataPtr->description = _description;
  this->dataPtr->type = _type;
  this->dataPtr->name = _name;
  this->dataPtr->sdf = NULL;
  this->dataPtr->node = NULL;

  // Record current world state
  this->dataPtr->startState = WorldState(this->dataPtr->world);
}

/////////////////////////////////////////////////
void UserCmd::Undo()
{
  // Record / override the state for redo
  this->dataPtr->endState = WorldState(this->dataPtr->world);

  // Set the world state
  this->dataPtr->world->SetState(this->dataPtr->startState);
}

/////////////////////////////////////////////////
void UserCmd::Redo()
{
  // Set the world state
  this->dataPtr->world->SetState(this->dataPtr->endState);
}

/////////////////////////////////////////////////
std::string UserCmd::Id()
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
std::string UserCmd::Description()
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
msgs::UserCmd::Type UserCmd::Type()
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
UserCmdManager::UserCmdManager(const WorldPtr _world)
  : dataPtr(new UserCmdManagerPrivate())
{
  this->dataPtr->world = _world;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->userCmdSub = this->dataPtr->node->Subscribe("~/user_cmd",
      &UserCmdManager::OnUserCmdMsg, this, true);

  this->dataPtr->undoRedoSub = this->dataPtr->node->Subscribe("~/undo_redo",
      &UserCmdManager::OnUndoRedoMsg, this);

  this->dataPtr->userCmdStatsPub =
    this->dataPtr->node->Advertise<msgs::UserCmdStats>("~/user_cmd_stats");
}

/////////////////////////////////////////////////
UserCmdManager::~UserCmdManager()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void UserCmdManager::OnUserCmdMsg(ConstUserCmdPtr &_msg)
{
  std::string name;
  if (_msg->has_entity_name())
    name = _msg->entity_name();

  UserCmd *cmd = new UserCmd(
      _msg->id(),
      this->dataPtr->world,
      _msg->description(),
      _msg->type(),
      name);

  // Add it to undo list
  this->dataPtr->undoCmds.push_back(cmd);

  // Clear redo list
  this->dataPtr->redoCmds.clear();

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
    UserCmd *cmd = this->dataPtr->undoCmds.back();

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

    // Undo all commands up to the one desired
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
    UserCmd *cmd = this->dataPtr->redoCmds.back();

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

    // Redo all commands up to the one desired
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


