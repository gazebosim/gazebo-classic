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

#ifndef _PUBLICATIONTRANSPORT_HH_
#define _PUBLICATIONTRANSPORT_HH_

#include <boost/shared_ptr.hpp>
#include <string>

#include "gazebo/transport/Connection.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class PublicationTransport PublicationTransport.hh
    /// transport/transport.hh
    /// \brief Reads data from a remote advertiser, and passes the data
    /// along to local subscribers
    class GAZEBO_VISIBLE PublicationTransport
    {
      /// \brief Constructor
      /// \param[in] _topic Topic that we're publishing
      /// \param[in] _topic Type of the topic that we're publishing
      public: PublicationTransport(const std::string &_topic,
                                   const std::string &_msgType);

      /// \brief Destructor
      public: virtual ~PublicationTransport();

      /// \brief Initialize the transport
      /// \param[in] _conn The underlying connection.
      /// \param[in] _latched True to grab the last message sent on the
      /// topic.
      public: void Init(const ConnectionPtr &_conn, bool _latched);

      /// \brief Finalize the transport
      public: void Fini();

      /// \brief Add a callback to the transport
      /// \param[in] _cb The callback to be added
      public: void AddCallback(
                  const boost::function<void(const std::string &)> &_cb);

      /// \brief Get the underlying connection
      /// \return Pointer to the underlying connection
      public: const ConnectionPtr GetConnection() const;

      /// \brief Get the topic name
      /// \return The topic name
      public: std::string GetTopic() const;

      /// \brief Get the topic type
      /// \return The topic type
      public: std::string GetMsgType() const;

      /// \brief Called when data is published.
      /// \param[in] _data Data to be published.
      private: void OnPublish(const std::string &_data);

      /// \brief The topic for this publication transport.
      private: std::string topic;

      /// \brief The type of messages that can be processed.
      private: std::string msgType;

      /// \brief The connection for the publication transport
      private: ConnectionPtr connection;

      /// \brief Callback used when OnPublish is called.
      private: boost::function<void (const std::string &)> callback;

      /// \brief Counter to give the publication transport a unique id.
      private: static int counter;

      /// \brief The unique id for the publication transport.
      private: int id;
    };
    /// \}
  }
}
#endif
