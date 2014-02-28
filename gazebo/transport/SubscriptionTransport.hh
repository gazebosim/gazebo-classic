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
#ifndef _SUBSCRIPTIONTRANSPORT_HH_
#define _SUBSCRIPTIONTRANSPORT_HH_

#include <boost/shared_ptr.hpp>
#include <string>

#include "Connection.hh"
#include "CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class SubscriptionTransport SubscriptionTransport.hh
    /// transport/transport.hh
    /// \brief Handles sending data over the wire to
    /// remote subscribers
    class SubscriptionTransport : public CallbackHelper
    {
      /// \brief Constructor
      public: SubscriptionTransport();

      /// \brief Destructor
      public: virtual ~SubscriptionTransport();

      /// \brief Initialize the publication link
      /// \param[in] _conn The connection to use
      /// \param[in] _latching If true, latch the latest message; if false,
      /// don't latch
      public: void Init(ConnectionPtr _conn, bool _latching);

      /// \brief Output a message to a connection
      /// \param[in] _newdata The message to be handled
      /// \return true if the message was handled successfully, false otherwise
      /// \param[in] _cb If non-null, callback to be invoked after
      /// transmission is complete.
      /// \param[in] _id ID associated with the message data.
      public: virtual bool HandleData(const std::string &_newdata,
                  boost::function<void(uint32_t)> _cb, uint32_t _id);

      // Documentation inherited
      public: virtual bool HandleMessage(MessagePtr _newMsg);

      /// \brief Get the connection we're using
      /// \return Pointer to the connection we're using
      public: const ConnectionPtr &GetConnection() const;

      /// \brief Is the callback local?
      /// \return true if the callback is local, false if the callback
      /// is tied to a  remote connection
      public: virtual bool IsLocal() const;

      private: ConnectionPtr connection;
    };
    /// \}
  }
}

#endif
