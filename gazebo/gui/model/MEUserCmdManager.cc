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

#include <boost/range/adaptor/reversed.hpp>

#include "gazebo/transport/Node.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/model/MEUserCmdManagerPrivate.hh"
#include "gazebo/gui/model/MEUserCmdManager.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
MEUserCmd::MEUserCmd(unsigned int _id,
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
  if (this->dataPtr->type == MEUserCmd::INSERTING_NESTED_MODEL &&
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
  if (this->dataPtr->type == MEUserCmd::INSERTING_NESTED_MODEL &&
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
MEUserCmd::CmdType MEUserCmd::Type() const
{
  return this->dataPtr->type;
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
MEUserCmdManager::MEUserCmdManager()
  : dataPtr(new MEUserCmdManagerPrivate)
{
  if (!g_undoAct || !g_redoAct || !g_undoHistoryAct || !g_redoHistoryAct)
  {
    gzerr << "Action missing, not initializing MEUserCmdManager" << std::endl;
    return;
  }

  this->dataPtr->idCounter = 0;

  // Action groups
  this->undoActions = new QActionGroup(this);
  this->undoActions->setExclusive(false);

  this->redoActions = new QActionGroup(this);
  this->redoActions->setExclusive(false);

  connect(this->undoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnUndoCommand(QAction *)));
  connect(this->undoActions, SIGNAL(hovered(QAction *)), this,
      SLOT(OnUndoHovered(QAction *)));

  connect(this->redoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnRedoCommand(QAction *)));
  connect(this->redoActions, SIGNAL(hovered(QAction *)), this,
      SLOT(OnRedoHovered(QAction *)));
}

/////////////////////////////////////////////////
MEUserCmdManager::~MEUserCmdManager()
{
}

/////////////////////////////////////////////////
void MEUserCmdManager::Init()
{
gzdbg << "Init" << std::endl;
}

/////////////////////////////////////////////////
void MEUserCmdManager::Clear()
{
gzdbg << "Clear" << std::endl;
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

  // Update buttons
  this->UpdateStats();
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

  // Update buttons
  this->UpdateStats();
}

/////////////////////////////////////////////////
MEUserCmd *MEUserCmdManager::NewCmd(const std::string &_description,
    const MEUserCmd::CmdType _type)
{
  // Create command
  MEUserCmd *cmd = new MEUserCmd(
      this->dataPtr->idCounter++,
      _description,
      _type);

  // Add it to undo list
  this->dataPtr->undoCmds.push_back(cmd);

  // Clear redo list
  this->dataPtr->redoCmds.clear();

  // Update buttons
  this->UpdateStats();

  return cmd;
}

/////////////////////////////////////////////////
void MEUserCmdManager::UpdateStats()
{
  g_undoAct->setEnabled(this->dataPtr->undoCmds.size() > 0);
  g_redoAct->setEnabled(this->dataPtr->redoCmds.size() > 0);
  g_undoHistoryAct->setEnabled(this->dataPtr->undoCmds.size() > 0);
  g_redoHistoryAct->setEnabled(this->dataPtr->redoCmds.size() > 0);
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnUndoCmdHistory()
{
  if (!this->Active())
    return;

  if (!this->undoActions)
  {
    gzerr << "No undo actions" << std::endl;
    return;
  }
  // Clear undo action group
  for (auto action : this->undoActions->actions())
  {
    this->undoActions->removeAction(action);
  }

  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(this->dataPtr->undoCmds))
  {
    QAction *action = new QAction(QString::fromStdString(cmd->Description()),
        this);
    action->setData(QVariant(cmd->Id()));
    action->setCheckable(true);
    menu.addAction(action);
    this->undoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCmdHistory()
{
  if (!this->Active())
    return;

  if (!this->redoActions)
  {
    gzerr << "No redo actions" << std::endl;
    return;
  }

  // Clear redo action group
  for (auto action : this->redoActions->actions())
  {
    this->redoActions->removeAction(action);
  }

  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(this->dataPtr->redoCmds))
  {
    QAction *action = new QAction(QString::fromStdString(cmd->Description()),
        this);
    action->setData(QVariant(cmd->Id()));
    action->setCheckable(true);
    menu.addAction(action);
    this->redoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

