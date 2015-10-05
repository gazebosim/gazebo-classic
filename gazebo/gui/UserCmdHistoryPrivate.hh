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
#ifndef _GAZEBO_USER_COMMAND_WIDGET_PRIVATE_HH_
#define _GAZEBO_USER_COMMAND_WIDGET_PRIVATE_HH_

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the UserCmdHistory class
    class UserCmdHistoryPrivate
    {
      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Publish undo / redo messages.
      public: transport::PublisherPtr undoRedoPub;

      /// \brief Subscriber to user command stats.
      public: transport::SubscriberPtr userCmdStatsSub;

      /// \brief Copy of last received user command stats message.
      public: msgs::UserCmdStats msg;

      /// \brief Group of actions in undo history menu.
      public: QActionGroup *undoActions;

      /// \brief Group of actions in redo history menu.
      public: QActionGroup *redoActions;
    };
  }
}
#endif

