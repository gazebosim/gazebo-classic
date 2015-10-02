/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

/// \brief A test class for the UserCmdHistory.
class UndoTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Response message received
  /// \brief _msg Message containing the response data
  private: void OnUndoRedo(ConstUndoRedoPtr &_msg);

  /// \brief Response message received
  /// \brief _msg Message containing the response data
  private: void OnUserCmdStats(ConstUserCmdStatsPtr &_msg);

  /// \brief Test receiving stats msgs and sending undo / redo requests.
  private slots: void MsgPassing();

  /// \brief Check that undo message has been received.
  private: bool g_undoMsgReceived = false;

  /// \brief Check that redo message has been received.
  private: bool g_redoMsgReceived = false;

  /// \brief Check that redo message has been received.
  private: int g_undoCmdCount = -1;
  private: int g_redoCmdCount = -1;
};

#endif
