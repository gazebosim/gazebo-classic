/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _WRENCHVISUAL_PRIVATE_HH_
#define _WRENCHVISUAL_PRIVATE_HH_

#include <vector>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \brief Private data for the Wrench Visual class.
    class WrenchVisualPrivate : public VisualPrivate
    {
      /// \brief Pointer to a node that handles communication.
      public: transport::NodePtr node;

      /// \brief Subscription to the sonar data.
      public: transport::SubscriberPtr wrenchSub;

      /// \brief Scene node for X torque visualization.
      public: Ogre::SceneNode *coneXNode;

      /// \brief Scene node for Y torque visualization.
      public: Ogre::SceneNode *coneYNode;

      /// \brief Scene node for Z torque visualization.
      public: Ogre::SceneNode *coneZNode;

      /// \brief Scene node for force visualization.
      public: Ogre::SceneNode *forceNode;

      /// \brief Line to visualize force
      public: DynamicLines *forceLine;

      /// \brief The current wrench message.
      public: boost::shared_ptr<msgs::WrenchStamped const> wrenchMsg;

      /// \brief True if we have received a message.
      public: bool receivedMsg;

      /// \brief True if this visualization is enabled.
      public: bool enabled;

      /// \brief Mutex to protect the contact message.
      public: boost::mutex mutex;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
