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

#ifndef _GAZEBO_UNDO_TEST_HH_
#define _GAZEBO_UNDO_TEST_HH_

#include <string>
#include "gazebo/gui/QTestFixture.hh"

class QtProperty;
class QtTreePropertyBrowser;

/// \brief Test undo / redo user commands.
class UndoTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Default constructor
  public: UndoTest() = default;

  /// \brief Test message passing between gui::UserCmdHistory and
  /// physics::UserCmdManager.
  private slots: void MsgPassing();

  /// \brief Test undoing translate model command.
  private slots: void UndoTranslateModel();

  /// \brief Test undoing rotate light command.
  private slots: void UndoRotateLight();

  /// \brief Test undoing scale model command.
  private slots: void UndoScaleModel();

  /// \brief Test undoing snap commands.
  private slots: void UndoSnap();

  /// \brief Test undoing align commands.
  private slots: void UndoAlign();

  /// \brief Test undoing reset time commands.
  private slots: void UndoResetTime();

  /// \brief Test undoing reset world commands.
  private slots: void UndoResetWorld();

  /// \brief Test undoing reset model poses commands.
  private slots: void UndoResetModelPoses();

  /// \brief Test undoing wrench commands.
  private slots: void UndoWrench();

  /// \brief UndoRedo message received
  /// \param[in] _msg Message containing an undo/redo request.
  private: void OnUndoRedo(ConstUndoRedoPtr &_msg);

  /// \brief UserCmdStats message received
  /// \param[in] _msg Message containing statistics about user commands saved in
  /// the server.
  private: void OnUserCmdStats(ConstUserCmdStatsPtr &_msg);

  /// \brief Check that undo message has been received.
  private: bool g_undoMsgReceived = false;

  /// \brief Check that redo message has been received.
  private: bool g_redoMsgReceived = false;

  /// \brief Number of undo commands in the stats msg.
  private: int g_undoCmdCount = -1;

  /// \brief Number of redo commands in the stats msg.
  private: int g_redoCmdCount = -1;
};

#endif
