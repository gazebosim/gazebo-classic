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
#ifndef _GAZEBO_USER_CMD_MANAGER_HH_
#define _GAZEBO_USER_CMD_MANAGER_HH_

#include <string>

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class UserCmdPrivate;

    /// \brief Class which represents a user command, which can be "undone"
    /// and "redone".
    class GAZEBO_VISIBLE UserCmd
    {
      /// \brief Constructor
      /// \param[in] _id Unique ID for this command
      /// \param[in] _world Pointer to the world
      /// \param[in] _description Description for the command, such as
      /// "Rotate box", "Delete sphere", etc.
      public: UserCmd(std::string _id,
                      physics::WorldPtr _world,
                      const std::string &_description,
                      msgs::UserCmd::Type _type,
                      const std::string &_name = "");

      /// \brief Destructor
      public: virtual ~UserCmd() = default;

      /// \brief Set the world state to the one just before the user command.
      public: virtual void Undo();

      /// \brief Set the world state to be that of the moment undo was last
      /// called.
      public: virtual void Redo();

      /// \brief Return this command's unique ID.
      /// \return Unique ID
      public: std::string Id();

      /// \brief Return this command's description
      /// \return Description
      public: std::string Description();

      /// \brief Return this command's description
      /// \return Description
      public: msgs::UserCmd::Type Type();

      /// \internal
      /// \brief Pointer to private data.
      protected: UserCmdPrivate *dataPtr;
    };

    class UserCmdManagerPrivate;

    /// \brief Manages user commands from the server side.
    class GAZEBO_VISIBLE UserCmdManager
    {
      /// \brief Constructor.
      public: UserCmdManager(const WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~UserCmdManager();

      /// \brief Callback when a UserCmd message is received.
      /// \param[in] _msg Incoming message
      private: void OnUserCmdMsg(ConstUserCmdPtr &_msg);

      /// \brief Callback when an UndoRedo message is received.
      /// \param[in] _msg Incoming message
      private: void OnUndoRedoMsg(ConstUndoRedoPtr &_msg);

      /// \brief Publish a message about current user command statistics.
      private: void PublishCurrentStats();

      /// \internal
      /// \brief Pointer to private data.
      private: UserCmdManagerPrivate *dataPtr;
    };
  }
}
#endif

