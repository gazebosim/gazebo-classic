/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/UserCmdHistoryPrivate.hh"
#include "gazebo/gui/UserCmdHistory.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
UserCmdHistory::UserCmdHistory()
  : dataPtr(new UserCmdHistoryPrivate)
{
  if (!g_undoAct || !g_redoAct || !g_undoHistoryAct || !g_redoHistoryAct)
  {
    gzerr << "Action missing, not initializing UserCmdHistory" << std::endl;
    return;
  }

  this->dataPtr->active = true;

  // Action groups
  this->undoActions = new QActionGroup(this);
  this->undoActions->setExclusive(false);

  this->redoActions = new QActionGroup(this);
  this->redoActions->setExclusive(false);

  // Pub / sub
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->undoRedoPub =
      this->dataPtr->node->Advertise<msgs::UndoRedo>("~/undo_redo");
  this->dataPtr->userCmdStatsSub =
      this->dataPtr->node->Subscribe("~/user_cmd_stats",
      &UserCmdHistory::OnUserCmdStatsMsg, this);

  // Qt connections
  connect(this, SIGNAL(StatsSignal()), this, SLOT(OnStatsSlot()));

  connect(g_undoAct, SIGNAL(triggered()), this, SLOT(OnUndo()));
  connect(g_redoAct, SIGNAL(triggered()), this, SLOT(OnRedo()));
  connect(g_undoHistoryAct, SIGNAL(triggered()), this,
      SLOT(OnUndoCmdHistory()));
  connect(g_redoHistoryAct, SIGNAL(triggered()), this,
      SLOT(OnRedoCmdHistory()));

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
UserCmdHistory::~UserCmdHistory()
{
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUndo()
{
  if (!this->dataPtr->active)
    return;

  this->OnUndoCommand(NULL);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUndoCommand(QAction *_action)
{
  if (!this->dataPtr->active)
    return;

  msgs::UndoRedo msg;
  msg.set_undo(true);

  if (_action)
  {
    msg.set_id(_action->data().toUInt());
  }

  this->dataPtr->undoRedoPub->Publish(msg);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUndoHovered(QAction *_action)
{
  bool beforeThis = true;
  for (auto action : this->undoActions->actions())
  {
    action->blockSignals(true);
    action->setChecked(beforeThis);
    action->blockSignals(false);

    if (action->data() == _action->data())
      beforeThis = false;
  }
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedo()
{
  if (!this->dataPtr->active)
    return;

  this->OnRedoCommand(NULL);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedoCommand(QAction *_action)
{
  if (!this->dataPtr->active)
    return;

  msgs::UndoRedo msg;
  msg.set_undo(false);

  if (_action)
  {
    msg.set_id(_action->data().toUInt());
  }

  this->dataPtr->undoRedoPub->Publish(msg);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedoHovered(QAction *_action)
{
  bool beforeThis = true;
  for (auto action : this->redoActions->actions())
  {
    action->blockSignals(true);
    action->setChecked(beforeThis);
    action->blockSignals(false);

    if (action->data() == _action->data())
      beforeThis = false;
  }
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUserCmdStatsMsg(ConstUserCmdStatsPtr &_msg)
{
  this->dataPtr->msg.Clear();
  this->dataPtr->msg.CopyFrom(*_msg);

  this->StatsSignal();
}

/////////////////////////////////////////////////
void UserCmdHistory::OnStatsSlot()
{
  g_undoAct->setEnabled(this->dataPtr->msg.undo_cmd_count() > 0);
  g_redoAct->setEnabled(this->dataPtr->msg.redo_cmd_count() > 0);
  g_undoHistoryAct->setEnabled(this->dataPtr->msg.undo_cmd_count() > 0);
  g_redoHistoryAct->setEnabled(this->dataPtr->msg.redo_cmd_count() > 0);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUndoCmdHistory()
{
  if (!this->dataPtr->active)
    return;

  // Clear undo action group
  for (auto action : this->undoActions->actions())
  {
    this->undoActions->removeAction(action);
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
    this->undoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedoCmdHistory()
{
  if (!this->dataPtr->active)
    return;

  // Clear redo action group
  for (auto action : this->redoActions->actions())
  {
    this->redoActions->removeAction(action);
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
    this->redoActions->addAction(action);
  }

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void UserCmdHistory::SetActive(const bool _active)
{
  this->dataPtr->active = _active;

  if (_active)
    this->StatsSignal();
}

/////////////////////////////////////////////////
bool UserCmdHistory::Active() const
{
  return this->dataPtr->active;
}

