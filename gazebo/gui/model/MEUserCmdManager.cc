/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/transport/Node.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/model/MEUserCmdManager.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the MEUserCmd class
    class MEUserCmdPrivate
    {
      /// \brief Unique ID identifying this command in the server.
      public: unsigned int id;

      /// \brief Description for the command.
      public: std::string description;

      /// \brief Type of command, such as MOVING or DELETING.
      public: MEUserCmd::CmdType type;

      /// \brief SDF element with information about the entity.
      public: sdf::ElementPtr sdf;

      /// \brief Fully scoped name of the entity involved in the command.
      public: std::string scopedName;

      /// \brief If the command is related to a joint, this is its unique Id.
      /// It's different from the scopedName and we need both.
      public: std::string jointId;

      /// \brief Pose before the command (to be used by undo).
      public: ignition::math::Pose3d poseBefore;

      /// \brief Pose after the command (to be used by redo).
      public: ignition::math::Pose3d poseAfter;

      /// \brief Map of scale for each visual and collision before the command
      /// (to be used by undo), indexed by the visual name.
      public: std::map<std::string, ignition::math::Vector3d> scalesBefore;

      /// \brief Map of scale for each visual and collision after the command
      /// (to be used by redo), indexed by the visual name.
      public: std::map<std::string, ignition::math::Vector3d> scalesAfter;
    };

    /// \internal
    /// \brief Private data for the MEUserCmdManager class
    class MEUserCmdManagerPrivate
    {
      /// \brief Counter to give commands unique ids.
      public: unsigned int idCounter;

      /// \brief List of commands which can be undone.
      public: std::vector<MEUserCmdPtr> undoCmds;

      /// \brief List of commands which can be redone.
      public: std::vector<MEUserCmdPtr> redoCmds;
    };
  }
}

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
MEUserCmd::MEUserCmd(const unsigned int _id,
    const std::string &_description,
    MEUserCmd::CmdType _type)
  : dataPtr(new MEUserCmdPrivate())
{
  this->dataPtr->id = _id;
  this->dataPtr->description = _description;
  this->dataPtr->type = _type;
  this->dataPtr->sdf = NULL;
  this->dataPtr->scopedName = "";
}

/////////////////////////////////////////////////
MEUserCmd::~MEUserCmd()
{
}

/////////////////////////////////////////////////
void MEUserCmd::Undo()
{
  // Inserting link
  if (this->dataPtr->type == MEUserCmd::INSERTING_LINK &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkRemoval(this->dataPtr->scopedName);
  }
  // Deleting link
  else if (this->dataPtr->type == MEUserCmd::DELETING_LINK &&
      this->dataPtr->sdf)
  {
    model::Events::requestLinkInsertion(this->dataPtr->sdf);
  }
  // Inserting nested model
  else if (this->dataPtr->type == MEUserCmd::INSERTING_NESTED_MODEL &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestNestedModelRemoval(this->dataPtr->scopedName);
  }
  // Deleting nested model
  else if (this->dataPtr->type == MEUserCmd::DELETING_NESTED_MODEL &&
      this->dataPtr->sdf)
  {
    model::Events::requestNestedModelInsertion(this->dataPtr->sdf);
  }
  // Inserting joint
  else if (this->dataPtr->type == MEUserCmd::INSERTING_JOINT &&
     !this->dataPtr->jointId.empty())
  {
    model::Events::requestJointRemoval(this->dataPtr->jointId);
  }
  // Deleting joint
  else if (this->dataPtr->type == MEUserCmd::DELETING_JOINT &&
      this->dataPtr->sdf)
  {
    auto topModelName = this->dataPtr->scopedName;
    size_t pIdx = topModelName.find("::");
    if (pIdx != std::string::npos)
      topModelName = topModelName.substr(0, pIdx);

    model::Events::requestJointInsertion(this->dataPtr->sdf, topModelName);
  }
  // Moving a link
  else if (this->dataPtr->type == MEUserCmd::MOVING_LINK &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkMove(this->dataPtr->scopedName,
        this->dataPtr->poseBefore);
  }
  // Moving a nested model
  else if (this->dataPtr->type == MEUserCmd::MOVING_NESTED_MODEL &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestNestedModelMove(this->dataPtr->scopedName,
        this->dataPtr->poseBefore);
  }
  // Scaling a link
  else if (this->dataPtr->type == MEUserCmd::SCALING_LINK &&
           !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkScale(this->dataPtr->scopedName,
        this->dataPtr->scalesBefore);
  }
  // Inserting model plugin
  else if (this->dataPtr->type == MEUserCmd::INSERTING_MODEL_PLUGIN &&
     !this->dataPtr->scopedName.empty())
  {
    model::Events::requestModelPluginRemoval(this->dataPtr->scopedName, false);
  }
  // Deleting model plugin
  else if (this->dataPtr->type == MEUserCmd::DELETING_MODEL_PLUGIN &&
      this->dataPtr->sdf)
  {
    auto pluginMsg = msgs::PluginFromSDF(this->dataPtr->sdf);

    model::Events::requestModelPluginInsertion(pluginMsg.name(),
        pluginMsg.filename(), pluginMsg.innerxml(), false);
  }
}

/////////////////////////////////////////////////
void MEUserCmd::Redo()
{
  // Inserting link
  if (this->dataPtr->type == MEUserCmd::INSERTING_LINK &&
      this->dataPtr->sdf)
  {
    model::Events::requestLinkInsertion(this->dataPtr->sdf);
  }
  // Deleting link
  else if (this->dataPtr->type == MEUserCmd::DELETING_LINK &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkRemoval(this->dataPtr->scopedName);
  }
  // Inserting nested model
  else if (this->dataPtr->type == MEUserCmd::INSERTING_NESTED_MODEL &&
      this->dataPtr->sdf)
  {
    model::Events::requestNestedModelInsertion(this->dataPtr->sdf);
  }
  // Deleting nested model
  else if (this->dataPtr->type == MEUserCmd::DELETING_NESTED_MODEL &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestNestedModelRemoval(this->dataPtr->scopedName);
  }
  // Inserting joint
  else if (this->dataPtr->type == MEUserCmd::INSERTING_JOINT &&
     this->dataPtr->sdf)
  {
    auto topModelName = this->dataPtr->scopedName;
    size_t pIdx = topModelName.find("::");
    if (pIdx != std::string::npos)
      topModelName = topModelName.substr(0, pIdx);

    model::Events::requestJointInsertion(this->dataPtr->sdf, topModelName);
  }
  // Deleting joint
  else if (this->dataPtr->type == MEUserCmd::DELETING_JOINT &&
     !this->dataPtr->jointId.empty())
  {
    model::Events::requestJointRemoval(this->dataPtr->jointId);
  }
  // Moving a link
  else if (this->dataPtr->type == MEUserCmd::MOVING_LINK &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkMove(this->dataPtr->scopedName,
        this->dataPtr->poseAfter);
  }
  // Moving a nested model
  else if (this->dataPtr->type == MEUserCmd::MOVING_NESTED_MODEL &&
      !this->dataPtr->scopedName.empty())
  {
    model::Events::requestNestedModelMove(this->dataPtr->scopedName,
        this->dataPtr->poseAfter);
  }
  // Scaling a link
  else if (this->dataPtr->type == MEUserCmd::SCALING_LINK &&
           !this->dataPtr->scopedName.empty())
  {
    model::Events::requestLinkScale(this->dataPtr->scopedName,
        this->dataPtr->scalesAfter);
  }
  // Inserting model plugin
  else if (this->dataPtr->type == MEUserCmd::INSERTING_MODEL_PLUGIN &&
     !this->dataPtr->scopedName.empty())
  {
    auto pluginMsg = msgs::PluginFromSDF(this->dataPtr->sdf);

    model::Events::requestModelPluginInsertion(pluginMsg.name(),
        pluginMsg.filename(), pluginMsg.innerxml(), false);
  }
  // Deleting model plugin
  else if (this->dataPtr->type == MEUserCmd::DELETING_MODEL_PLUGIN &&
      this->dataPtr->sdf)
  {
    model::Events::requestModelPluginRemoval(this->dataPtr->scopedName, false);
  }
}

/////////////////////////////////////////////////
unsigned int MEUserCmd::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
std::string MEUserCmd::Description() const
{
  return this->dataPtr->description;
}

/////////////////////////////////////////////////
void MEUserCmd::SetSDF(sdf::ElementPtr _sdf)
{
  this->dataPtr->sdf = _sdf;
}

/////////////////////////////////////////////////
void MEUserCmd::SetScopedName(const std::string &_name)
{
  this->dataPtr->scopedName = _name;
}

/////////////////////////////////////////////////
void MEUserCmd::SetJointId(const std::string &_id)
{
  this->dataPtr->jointId = _id;
}

/////////////////////////////////////////////////
void MEUserCmd::SetPoseChange(const ignition::math::Pose3d &_before,
    const ignition::math::Pose3d &_after)
{
  this->dataPtr->poseBefore = _before;
  this->dataPtr->poseAfter = _after;
}

/////////////////////////////////////////////////
void MEUserCmd::SetScaleChange(
    const std::map<std::string, ignition::math::Vector3d> &_before,
    const std::map<std::string, ignition::math::Vector3d> &_after)
{
  if (_before.size() != _after.size())
  {
    gzwarn << "Number of scale operations before [" << _before.size()
      << "] and after [" << _after.size() << "] command are "
      << "different, some visuals or collisions might be scaled wrong."
      << std::endl;
  }

  this->dataPtr->scalesBefore = _before;
  this->dataPtr->scalesAfter = _after;
}

/////////////////////////////////////////////////
MEUserCmdManager::MEUserCmdManager()
  : dataPtr(new MEUserCmdManagerPrivate)
{
  if (!g_undoAct || !g_redoAct || !g_undoHistoryAct || !g_redoHistoryAct)
  {
    gzerr << "Action missing, not initializing MEUserCmdManager" << std::endl;
    this->SetActive(false);
    return;
  }

  this->dataPtr->idCounter = 0;

  this->SetActive(false);
}

/////////////////////////////////////////////////
MEUserCmdManager::~MEUserCmdManager()
{
}

/////////////////////////////////////////////////
void MEUserCmdManager::Reset()
{
  this->dataPtr->undoCmds.clear();
  this->dataPtr->redoCmds.clear();
  this->StatsSignal();
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnUndoCommand(QAction *_action)
{
  if (!this->Active())
    return;

  if (this->dataPtr->undoCmds.empty())
  {
    gzwarn << "No commands to be undone" << std::endl;
    return;
  }

  // Get the last done command
  auto cmd = this->dataPtr->undoCmds.back();

  // If there's an action, get that command instead
  if (_action)
  {
    bool found = false;
    auto id = _action->data().toUInt();
    for (auto cmdIt : this->dataPtr->undoCmds)
    {
      if (cmdIt->Id() == id)
      {
        cmd = cmdIt;
        found = true;
        break;
      }
    }
    if (!found)
    {
      gzerr << "Requested command [" << id <<
          "] is not in the undo queue and won't be undone." << std::endl;
      return;
    }
  }

  // Undo all commands up to the desired one
  for (auto cmdIt = this->dataPtr->undoCmds.rbegin();
      cmdIt != this->dataPtr->undoCmds.rend(); ++cmdIt)
  {
    // Undo it
    (*cmdIt)->Undo();

    // Transfer to the redo list
    this->dataPtr->redoCmds.push_back(*cmdIt);
    this->dataPtr->undoCmds.pop_back();

    if ((*cmdIt) == cmd)
      break;
  }

  // Update buttons
  this->StatsSignal();
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCommand(QAction *_action)
{
  if (!this->Active())
    return;

  if (this->dataPtr->redoCmds.empty())
  {
    gzwarn << "No commands to be redone" << std::endl;
    return;
  }

  // Get the last done command
  auto cmd = this->dataPtr->redoCmds.back();

  // If there's an action, get that command instead
  if (_action)
  {
    bool found = false;
    auto id = _action->data().toUInt();
    for (auto cmdIt : this->dataPtr->redoCmds)
    {
      if (cmdIt->Id() == id)
      {
        cmd = cmdIt;
        found = true;
        break;
      }
    }
    if (!found)
    {
      gzerr << "Requested command [" << id <<
          "] is not in the redo queue and won't be redone." << std::endl;
      return;
    }
  }

  // Redo all commands up to the desired one
  for (auto cmdIt = this->dataPtr->redoCmds.rbegin();
      cmdIt != this->dataPtr->redoCmds.rend(); ++cmdIt)
  {
    // Redo it
    (*cmdIt)->Redo();

    // Transfer to the undo list
    this->dataPtr->undoCmds.push_back(*cmdIt);
    this->dataPtr->redoCmds.pop_back();

    if ((*cmdIt) == cmd)
      break;
  }

  // Update buttons
  this->StatsSignal();
}

/////////////////////////////////////////////////
MEUserCmdPtr MEUserCmdManager::NewCmd(
    const std::string &_description, const MEUserCmd::CmdType _type)
{
  // Create command
  MEUserCmdPtr cmd;
  cmd.reset(new MEUserCmd(this->dataPtr->idCounter++,
      _description, _type));

  // Add it to undo list
  this->dataPtr->undoCmds.push_back(cmd);

  // Clear redo list
  this->dataPtr->redoCmds.clear();

  // Update buttons
  this->StatsSignal();

  return cmd;
}

/////////////////////////////////////////////////
bool MEUserCmdManager::HasUndo() const
{
  return !this->dataPtr->undoCmds.empty();
}

/////////////////////////////////////////////////
bool MEUserCmdManager::HasRedo() const
{
  return !this->dataPtr->redoCmds.empty();
}

/////////////////////////////////////////////////
std::vector<std::pair<unsigned int, std::string>>
    MEUserCmdManager::Cmds(const bool _undo) const
{
  auto cmds = this->dataPtr->undoCmds;

  if (!_undo)
    cmds = this->dataPtr->redoCmds;

  std::vector<std::pair<unsigned int, std::string>> result;
  for (auto cmd : cmds)
  {
    result.push_back(std::pair<unsigned int, std::string>(
        cmd->Id(), cmd->Description()));
  }

  return result;
}


