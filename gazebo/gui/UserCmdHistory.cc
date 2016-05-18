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
  this->dataPtr->undoActions = new QActionGroup(this);
  this->dataPtr->undoActions->setExclusive(false);

  this->dataPtr->redoActions = new QActionGroup(this);
  this->dataPtr->redoActions->setExclusive(false);

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
UserCmdHistory::~UserCmdHistory()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
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
  if (!this->dataPtr->active)
    return;

  bool beforeThis = true;
  for (auto action : this->dataPtr->undoActions->actions())
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
  if (!this->dataPtr->active)
    return;

  bool beforeThis = true;
  for (auto action : this->dataPtr->redoActions->actions())
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
  if (!this->dataPtr->active)
    return;

  g_undoAct->setEnabled(this->HasUndo());
  g_redoAct->setEnabled(this->HasRedo());
  g_undoHistoryAct->setEnabled(this->HasUndo());
  g_redoHistoryAct->setEnabled(this->HasRedo());
}

/////////////////////////////////////////////////
bool UserCmdHistory::HasUndo() const
{
  return this->dataPtr->msg.undo_cmd_count() > 0;
}

/////////////////////////////////////////////////
bool UserCmdHistory::HasRedo() const
{
  return this->dataPtr->msg.redo_cmd_count() > 0;
}

/////////////////////////////////////////////////
std::vector<std::pair<unsigned int, std::string>>
    UserCmdHistory::Cmds(const bool _undo) const
{
  auto cmds = this->dataPtr->msg.undo_cmd();

  if (!_undo)
    cmds = this->dataPtr->msg.redo_cmd();

  std::vector<std::pair<unsigned int, std::string>> result;
  for (auto cmd : cmds)
  {
    result.push_back(std::pair<unsigned int, std::string>(
        cmd.id(), cmd.description()));
  }

  return result;
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUndoCmdHistory()
{
  this->OnCmdHistory(true);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedoCmdHistory()
{
  this->OnCmdHistory(false);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnCmdHistory(const bool _undo)
{
  if (!this->dataPtr->active)
    return;

  // Clear action group
  if (_undo)
  {
    for (auto action : this->dataPtr->undoActions->actions())
      this->dataPtr->undoActions->removeAction(action);
  }
  else
  {
    for (auto action : this->dataPtr->redoActions->actions())
      this->dataPtr->redoActions->removeAction(action);
  }

  // Create new menu
  QMenu menu;
  auto cmds = this->Cmds(_undo);
  for (auto cmd = cmds.rbegin(); cmd != cmds.rend(); ++cmd)
  {
    auto action = new QAction(QString::fromStdString((*cmd).second), this);
    action->setData(QVariant((*cmd).first));
    action->setCheckable(true);
    menu.addAction(action);

    if (_undo)
      this->dataPtr->undoActions->addAction(action);
    else
      this->dataPtr->redoActions->addAction(action);
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

