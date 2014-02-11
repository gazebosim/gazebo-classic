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

#ifndef _TRANSMITTERVISUAL_PRIVATE_HH_
#define _TRANSMITTERVISUAL_PRIVATE_HH_

#include <vector>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Transmitter Visual class.
    class TransmitterVisualPrivate : public VisualPrivate
    {
      /// \brief Pointer to a node that handles communication.
      public: transport::NodePtr node;

      /// \brief Subscription to the propagation data.
      public: transport::SubscriberPtr signalPropagationSub;

      /// \brief Renders the points representing the signal strength.
      public: DynamicLines *points;

      /// \brief Use for allocate the visuals for the grid only the first time
      /// you receive the grid. The next times there are just updates.
      public: bool isFirst;

      /// \brief Store the list of visuals
      public: std::vector<rendering::VisualPtr> vectorLink;

       /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Mutex to protect the contact message.
      public: boost::mutex mutex;

      /// \brief The current contact message.
      public: boost::shared_ptr<msgs::PropagationGrid const> gridMsg;

      /// \brief True if we have received a message.
      public: bool receivedMsg;
    };
  }
}
#endif
