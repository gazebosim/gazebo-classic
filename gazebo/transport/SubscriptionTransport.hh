/*
 * Copyright 2011 Nate Koenig
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
#ifndef SUBSCRIPTIONTRANSPORT_HH
#define SUBSCRIPTIONTRANSPORT_HH

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

    /// \brief Handles sending data over the wire to remote subscribers
    class SubscriptionTransport : public CallbackHelper
    {
      /// \brief Constructor
      public: SubscriptionTransport();

      /// \brief Destructor
      public: virtual ~SubscriptionTransport();

      /// \brief Initialize the publication link
      public: void Init(const ConnectionPtr &conn, bool _latching);

      /// \brief Output a message to a connection
      public: virtual bool HandleData(const std::string &newdata);

      /// \brief Get the connection
      public: const ConnectionPtr &GetConnection() const;

      /// \brief Return true if the callback is local, false if the callback
      /// is tied to a  remote connection
      public: virtual bool IsLocal() const;

      private: ConnectionPtr connection;
    };
    /// \}
  }
}

#endif
