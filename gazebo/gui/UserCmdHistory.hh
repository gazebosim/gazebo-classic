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

#ifndef _GAZEBO_USER_COMMAND_WIDGET_HH_
#define _GAZEBO_USER_COMMAND_WIDGET_HH_

#include <string>

#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class UserCmdHistoryPrivate;

    /// \brief Class which manages user commands in the client side.
    class GZ_GUI_VISIBLE UserCmdHistory : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: UserCmdHistory();

      /// \brief Destructor
      public: virtual ~UserCmdHistory();

      /// \internal
      /// \brief Triggers OnStatsSlot
      signals: void StatsSignal();

      /// \brief Qt call back when the undo button is pressed.
      private slots: void OnUndo();

      /// \brief Qt call back when an undo history action is triggered.
      /// It publishes an undo request message.
      private slots: void OnUndoCommand(QAction *_action);

      /// \brief Qt call back when an undo history action is hovered.
      private slots: void OnUndoHovered(QAction *_action);

      /// \brief Qt call back when the undo history button is pressed.
      /// It opens the undo history menu.
      private slots: void OnUndoCmdHistory();

      /// \brief Qt call back when the redo button is pressed.
      private slots: void OnRedo();

      /// \brief Qt call back when a redo history action is triggered.
      /// It publishes a redo request message.
      private slots: void OnRedoCommand(QAction *_action);

      /// \brief Qt call back when a redo history action is hovered.
      private slots: void OnRedoHovered(QAction *_action);

      /// \brief Qt call back when the redo history button is pressed.
      /// It opens the redo history menu.
      private slots: void OnRedoCmdHistory();

      /// \brief Updates the widgets according to the user command stats
      /// message.
      private slots: void OnStatsSlot();

      /// \brief User command statistics message callback.
      /// \param[in] _msg Message containing statistics about user commands
      /// stored in the server.
      private: void OnUserCmdStatsMsg(ConstUserCmdStatsPtr &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: UserCmdHistoryPrivate *dataPtr;
    };
  }
}
#endif

