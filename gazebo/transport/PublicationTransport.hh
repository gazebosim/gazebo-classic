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

#ifndef PUBLICATIONTRANSPORT_HH
#define PUBLICATIONTRANSPORT_HH

#include <boost/shared_ptr.hpp>
#include <string>

#include "transport/Connection.hh"
#include "common/Event.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class PublicationTransport PublicationTransport.hh transport/transport.hh
    /// \brief Reads data from a remote advertiser, and passes the data
    /// along to local subscribers
    class PublicationTransport
    {
      /// \brief Constructor
      /// \param[in] _topic Topic that we're publishing
      /// \param[in] _topic Type of the topic that we're publishing
      public: PublicationTransport(const std::string &_topic,
                                   const std::string &_msgType);

      /// \brief Destructor
      public: virtual ~PublicationTransport();

      /// \brief Initialize the transport
      /// \param[in] _conn The underlying connection
      public: void Init(const ConnectionPtr &_conn);

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

      private: void OnConnectionShutdown();

      private: void OnPublish(const std::string &data);

      private: std::string topic;
      private: std::string msgType;
      private: ConnectionPtr connection;
      private: boost::function<void (const std::string &)> callback;
      private: event::ConnectionPtr shutdownConnectionPtr;

      private: static int counter;
      private: int id;
    };
    /// \}
  }
}

#endif


