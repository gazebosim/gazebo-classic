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

#ifndef _WRENCHVISUAL_HH_
#define _WRENCHVISUAL_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    class DynamicLines;

    /// \class WrenchVisual WrenchVisual.hh rendering/rendering.hh
    /// \brief Visualization for sonar data.
    class WrenchVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      /// \param[in] _topicName Name of the topic that has sonar data.
      public: WrenchVisual(const std::string &_name, VisualPtr _vis,
                          const std::string &_topicName);

      /// \brief Destructor.
      public: virtual ~WrenchVisual();

      /// \brief Load the visual based on a message
      /// \param[in] _msg Joint message
      public: void Load(ConstJointPtr &_msg);

      /// \brief Set to true to enable wrench visualization.
      /// \param[in] _enabled True to show wrenches, false to hide.
      public: void SetEnabled(bool _enabled);

      /// \brief Callback when sonar data is received.
      private: void OnMsg(ConstWrenchStampedPtr &_msg);

      /// \brief Update the wrench visual.
      private: void Update();

      /// \brief Pointer to a node that handles communication.
      private: transport::NodePtr node;

      /// \brief Subscription to the sonar data.
      private: transport::SubscriberPtr wrenchSub;

      /// \brief Scene node for X torque visualization.
      private: Ogre::SceneNode *coneXNode;

      /// \brief Scene node for Y torque visualization.
      private: Ogre::SceneNode *coneYNode;

      /// \brief Scene node for Z torque visualization.
      private: Ogre::SceneNode *coneZNode;

      /// \brief Scene node for force visualization.
      private: Ogre::SceneNode *forceNode;

      /// \brief Line to visualize force
      private: DynamicLines *forceLine;

      /// \brief The current wrench message.
      private: boost::shared_ptr<msgs::WrenchStamped const> wrenchMsg;

      /// \brief True if we have received a message.
      private: bool receivedMsg;

      /// \brief True if this visualization is enabled.
      private: bool enabled;

      /// \brief Mutex to protect the contact message.
      private: boost::mutex mutex;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
