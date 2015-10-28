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

#ifndef _SONARVISUAL_PRIVATE_HH_
#define _SONARVISUAL_PRIVATE_HH_

#include <vector>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \brief Private data for the Sonar Visual class.
    class SonarVisualPrivate : public VisualPrivate
    {
      /// \brief Pointer to a node that handles communication.
      public: transport::NodePtr node;

      /// \brief Subscription to the sonar data.
      public: transport::SubscriberPtr sonarSub;

      /// \brief Renders the sonar data reading.
      public: DynamicLines *sonarRay;

      /// \brief Renders the sonar cone.
      public: Ogre::SceneNode *coneNode;

      /// \brief The current sonar message.
      public: boost::shared_ptr<msgs::SonarStamped const> sonarMsg;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Mutex to protect the contact message.
      public: boost::mutex mutex;

      /// \brief True if we have received a message.
      public: bool receivedMsg;
    };
  }
}
#endif
