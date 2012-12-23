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
/* Desc: Handles a subscription to a topic
 * Author: Nate Koenig
 */

#ifndef _SUBSCRIBER_HH_
#define _SUBSCRIBER_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include "transport/CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class Subscriber Subscriber.hh transport/transport.hh
    /// \brief A subscriber to a topic
    class Subscriber
    {
      /// \brief Constructor
      /// \param[in] _topic The topic we're subscribing to
      /// \param[in] _node The associated node
      public: Subscriber(const std::string &_topic, NodePtr _node);

      /// \brief Destructor
      public: virtual ~Subscriber();

      /// \brief Get the topic name
      /// \return The topic name
      public: std::string GetTopic() const;

      /// \brief Unsubscribe from the topic
      public: void Unsubscribe() const;

      private: std::string topic;
      private: NodePtr node;
    };
    /// \}
  }
}

#endif


