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

  if (g_undoAct && g_redoAct && g_undoHistoryAct && g_redoHistoryAct)
  {
    connect(g_undoAct, SIGNAL(triggered()), this, SLOT(OnUndo()));
    connect(g_redoAct, SIGNAL(triggered()), this, SLOT(OnRedo()));
    connect(g_undoHistoryAct, SIGNAL(triggered()), this, SLOT(OnCmdHistory()));
  }
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
  msgs::UndoRedo msg;
  msg.set_undo(true);
  // ID
  this->dataPtr->undoRedoPub->Publish(msg);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnRedo()
{
  msgs::UndoRedo msg;
  msg.set_undo(false);
  // ID
  this->dataPtr->undoRedoPub->Publish(msg);
}

/////////////////////////////////////////////////
void UserCmdHistory::OnUserCmdStatsMsg(ConstUserCmdStatsPtr &_msg)
{
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
void UserCmdHistory::OnCmdHistory()
{
  QMenu menu;

  for (auto cmd : this->dataPtr->msg.undo_cmd())
  {
    QAction *action = new QAction(QString::fromStdString(cmd.description()),
        this);
    menu.addAction(action);
  }

  menu.addSeparator();

  for (auto cmd : this->dataPtr->msg.redo_cmd())
  {
    QAction *action = new QAction(QString::fromStdString(cmd.description()),
        this);
    menu.addAction(action);
  }

  menu.exec(QCursor::pos());
}

