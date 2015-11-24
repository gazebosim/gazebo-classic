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

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
MEUserCmd::MEUserCmd(unsigned int _id,
                 const std::string &_description,
                 msgs::UserCmd::Type _type)
  : dataPtr(new MEUserCmdPrivate())
{
  this->dataPtr->id = _id;
  this->dataPtr->description = _description;
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
MEUserCmd::~MEUserCmd()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void MEUserCmd::Undo()
{
  if (this->dataPtr->type == msgs::UserCmd::INSERTING)
  {
    gzdbg << "Undo insert command: " << this->Description() << std::endl;
  }
}

/////////////////////////////////////////////////
void MEUserCmd::Redo()
{
  if (this->dataPtr->type == msgs::UserCmd::INSERTING)
  {
    gzdbg << "Redo insert command: " << this->Description() << std::endl;
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
msgs::UserCmd::Type MEUserCmd::Type() const
{
  return this->dataPtr->type;
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
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);


  dPtr->idCounter = 0;

  // Action groups
  dPtr->undoActions = new QActionGroup(this);
  dPtr->undoActions->setExclusive(false);

  dPtr->redoActions = new QActionGroup(this);
  dPtr->redoActions->setExclusive(false);

  connect(dPtr->undoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnUndoCommand(QAction *)));
  connect(dPtr->undoActions, SIGNAL(hovered(QAction *)), this,
      SLOT(OnUndoHovered(QAction *)));

  connect(dPtr->redoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnRedoCommand(QAction *)));
  connect(dPtr->redoActions, SIGNAL(hovered(QAction *)), this,
      SLOT(OnRedoHovered(QAction *)));
}

/////////////////////////////////////////////////
MEUserCmdManager::~MEUserCmdManager()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
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
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);

  if (dPtr->undoCmds.empty())
  {
    gzwarn << "No commands to be undone" << std::endl;
    return;
  }

  // Get the last done command
  auto cmd = dPtr->undoCmds.back();

  // If there's an action, get that command instead
  if (_action)
  {
    bool found = false;
    auto id = _action->data().toUInt();
    for (auto cmdIt : dPtr->undoCmds)
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
  for (auto cmdIt : boost::adaptors::reverse(dPtr->undoCmds))
  {
    // Undo it
    cmdIt->Undo();

    // Transfer to the redo list
    dPtr->undoCmds.pop_back();
    dPtr->redoCmds.push_back(cmdIt);

    if (cmdIt == cmd)
      break;
  }

  // Update buttons
  this->UpdateStats();
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCommand(QAction *_action)
{
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);

  if (dPtr->redoCmds.empty())
  {
    gzwarn << "No commands to be redone" << std::endl;
    return;
  }

  // Get the last done command
  auto cmd = dPtr->redoCmds.back();

  // If there's an action, get that command instead
  if (_action)
  {
    bool found = false;
    auto id = _action->data().toUInt();
    for (auto cmdIt : dPtr->redoCmds)
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
  for (auto cmdIt : boost::adaptors::reverse(dPtr->redoCmds))
  {
    // Redo it
    cmdIt->Redo();

    // Transfer to the undo list
    dPtr->redoCmds.pop_back();
    dPtr->undoCmds.push_back(cmdIt);

    if (cmdIt == cmd)
      break;
  }

  // Update buttons
  this->UpdateStats();
}

/////////////////////////////////////////////////
void MEUserCmdManager::NewCmd(const std::string &_description,
    const msgs::UserCmd::Type _type)
{
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);

  // Create command
  MEUserCmd *cmd = new MEUserCmd(
      dPtr->idCounter++,
      _description,
      _type);

  // Add it to undo list
  dPtr->undoCmds.push_back(cmd);

  // Clear redo list
  dPtr->redoCmds.clear();

  // Update buttons
  this->UpdateStats();
}

/////////////////////////////////////////////////
void MEUserCmdManager::UpdateStats()
{
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);
  g_undoAct->setEnabled(dPtr->undoCmds.size() > 0);
  g_redoAct->setEnabled(dPtr->redoCmds.size() > 0);
  g_undoHistoryAct->setEnabled(dPtr->undoCmds.size() > 0);
  g_redoHistoryAct->setEnabled(dPtr->redoCmds.size() > 0);
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnUndoCmdHistory()
{
  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);
  if (!dPtr->undoActions)
  {
    gzerr << "No undo actions" << std::endl;
    return;
  }
  // Clear undo action group
  for (auto action : dPtr->undoActions->actions())
  {
    dPtr->undoActions->removeAction(action);
  }


  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(dPtr->undoCmds))
  {
    QAction *action = new QAction(QString::fromStdString(cmd->Description()),
        this);
    action->setData(QVariant(cmd->Id()));
    action->setCheckable(true);
    menu.addAction(action);
    dPtr->undoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCmdHistory()
{
  // Clear redo action group
  for (auto action : this->dataPtr->redoActions->actions())
  {
    this->dataPtr->redoActions->removeAction(action);
  }

  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);

  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(dPtr->redoCmds))
  {
    QAction *action = new QAction(QString::fromStdString(cmd->Description()),
        this);
    action->setData(QVariant(cmd->Id()));
    action->setCheckable(true);
    menu.addAction(action);
    this->dataPtr->redoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

