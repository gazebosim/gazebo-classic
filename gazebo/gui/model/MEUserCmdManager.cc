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
MEUserCmd::MEUserCmd(std::string _id,
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
}

/////////////////////////////////////////////////
void MEUserCmd::Redo()
{
}

/////////////////////////////////////////////////
std::string MEUserCmd::Id() const
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

  // Action groups
  this->dataPtr->undoActions = new QActionGroup(this);
  this->dataPtr->undoActions->setExclusive(false);

  this->dataPtr->redoActions = new QActionGroup(this);
  this->dataPtr->redoActions->setExclusive(false);

  // Qt connections
  connect(this, SIGNAL(StatsSignal()), this, SLOT(OnStatsSlot()));

  connect(g_undoAct, SIGNAL(triggered()), this, SLOT(OnUndo()));
  connect(g_redoAct, SIGNAL(triggered()), this, SLOT(OnRedo()));
  connect(g_undoHistoryAct, SIGNAL(triggered()), this,
      SLOT(OnUndoCmdHistory()));
  connect(g_redoHistoryAct, SIGNAL(triggered()), this,
      SLOT(OnRedoCmdHistory()));

  connect(this->dataPtr->undoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnUndoCommand(QAction *)));
  connect(this->dataPtr->undoActions, SIGNAL(hovered(QAction *)), this,
      SLOT(OnUndoHovered(QAction *)));

  connect(this->dataPtr->redoActions, SIGNAL(triggered(QAction *)), this,
      SLOT(OnRedoCommand(QAction *)));
  connect(this->dataPtr->redoActions, SIGNAL(hovered(QAction *)), this,
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
  // Directly call undo on command
gzdbg << "OnUndoCmd" << std::endl;
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCommand(QAction *_action)
{
  // Directly call redo on command
gzdbg << "OnRedoCmd" << std::endl;
}

/////////////////////////////////////////////////
void MEUserCmdManager::NewCmd(const std::string &_id,
                 const std::string &_description,
                 const msgs::UserCmd::Type _type)
{
  // Create command
  MEUserCmd *cmd = new MEUserCmd(
      _id,
      _description,
      _type);


  auto *dPtr = reinterpret_cast<MEUserCmdManagerPrivate *>(this->dataPtr);

  // Add it to undo list
  dPtr->undoCmds.push_back(cmd);

  // Clear redo list
  dPtr->redoCmds.clear();
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnStatsSlot()
{
  g_undoAct->setEnabled(this->dataPtr->msg.undo_cmd_count() > 0);
  g_redoAct->setEnabled(this->dataPtr->msg.redo_cmd_count() > 0);
  g_undoHistoryAct->setEnabled(this->dataPtr->msg.undo_cmd_count() > 0);
  g_redoHistoryAct->setEnabled(this->dataPtr->msg.redo_cmd_count() > 0);
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnUndoCmdHistory()
{
gzdbg << "OnUndoCmdHistory" << std::endl;
  // Clear undo action group
  for (auto action : this->dataPtr->undoActions->actions())
  {
    this->dataPtr->undoActions->removeAction(action);
  }

  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(this->dataPtr->msg.undo_cmd()))
  {
    QAction *action = new QAction(QString::fromStdString(cmd.description()),
        this);
    action->setData(QVariant(cmd.id()));
    action->setCheckable(true);
    menu.addAction(action);
    this->dataPtr->undoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void MEUserCmdManager::OnRedoCmdHistory()
{
gzdbg << "OnRedoCmdHistory" << std::endl;
  // Clear redo action group
  for (auto action : this->dataPtr->redoActions->actions())
  {
    this->dataPtr->redoActions->removeAction(action);
  }

  // Create new menu
  QMenu menu;
  for (auto cmd : boost::adaptors::reverse(this->dataPtr->msg.redo_cmd()))
  {
    QAction *action = new QAction(QString::fromStdString(cmd.description()),
        this);
    action->setData(QVariant(cmd.id()));
    action->setCheckable(true);
    menu.addAction(action);
    this->dataPtr->redoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

