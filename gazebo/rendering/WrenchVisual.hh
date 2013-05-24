/*
 * Copyright 2012 Open Source Robotics Foundation
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

      /// \brief Callback when sonar data is received.
      private: void OnMsg(ConstWrenchStampedPtr &_msg);

      /// \brief Pointer to a node that handles communication.
      private: transport::NodePtr node;

      /// \brief Subscription to the sonar data.
      private: transport::SubscriberPtr wrenchSub;

      /// \brief Renders the sonar cone.
      private: Ogre::SceneNode *coneXNode;
      private: Ogre::SceneNode *coneYNode;
      private: Ogre::SceneNode *coneZNode;
    };
    /// \}
  }
}
#endif
