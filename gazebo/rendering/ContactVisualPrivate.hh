/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _CONTACTVISUAL_PRIVATE_HH_
#define _CONTACTVISUAL_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \brief Private data for the Arrow Visual class
    class ContactVisualPrivate : public VisualPrivate
    {
      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Subscription to the contact data.
      public: transport::SubscriberPtr contactsSub;

      /// \brief The current contact message.
      public: boost::shared_ptr<msgs::Contacts const> contactsMsg;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief A contact point visualization.
      public: class ContactPoint
               {
                 /// \brief The contact point visual
                 public: VisualPtr contactPointVis;
                 /// \brief Normal and depth for the contact point.
                 public: DynamicLines *normal, *depth;
               };

      /// \brief All the contact points.
      public: std::vector<ContactVisualPrivate::ContactPoint *> points;

      /// \brief Mutex to protect the contact message.
      public: boost::mutex mutex;

      /// \brief True if we have received a message.
      public: bool receivedMsg;

      /// \brief True if this visualization is enabled.
      public: bool enabled;

      /// \brief Name of the topic contact information is published on
      public: std::string topicName;
    };
  }
}
#endif
