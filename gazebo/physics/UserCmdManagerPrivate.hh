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
#ifndef _GAZEBO_USER_CMD_MANAGER_PRIVATE_HH_
#define _GAZEBO_USER_CMD_MANAGER_PRIVATE_HH_

#include <sdf/sdf.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class WorldState;

    /// \internal
    /// \brief Private data for the UserCmdManager class
    class UserCmdPrivate
    {
      /// \brief Pointer to the world.
      public: WorldPtr world;

      /// \brief SDF representing the whole world state the moment the user
      /// command was executed.
      public: WorldState startState;

      /// \brief SDF representing the whole world state for the most recent
      /// time the user has triggeded undo for this command.
      public: WorldState endState;

      /// \brief Unique ID identifying this command in the server. It is set by
      /// the client which generated the user command, so we must find a way to
      /// avoid collisions.
      public: std::string id;

      /// \brief Description for the command.
      public: std::string description;

      public: msgs::UserCmd::Type type;
      public: sdf::SDFPtr sdf;
      public: std::string name;

      /// \brief Node for communication.
      /// TODO: have node on manager instead, so it's only one
      public: transport::NodePtr node;

      /// \brief Publisher that publishes msg to spawn new model
      public: transport::PublisherPtr factoryPub;

      /// \brief Publisher that publishes msg to spawn new model
      public: transport::PublisherPtr lightPub;
    };

    class UserCmd;

    /// \internal
    /// \brief Private data for the UserCmdManager class
    class UserCmdManagerPrivate
    {
      /// \brief Pointer to the world.
      public: WorldPtr world;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Subscriber to user command messages.
      public: transport::SubscriberPtr userCmdSub;

      /// \brief Subscriber to undo redo messages.
      public: transport::SubscriberPtr undoRedoSub;

      /// \brief Publisher of undo redo statistics messages.
      public: transport::PublisherPtr userCmdStatsPub;

      public: std::vector<UserCmd *> undoCmds;
      public: std::vector<UserCmd *> redoCmds;
    };
  }
}
#endif

