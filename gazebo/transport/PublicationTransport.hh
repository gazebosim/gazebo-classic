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

    /// \brief Reads data from a remote advertiser, and passes the data
    /// along to local subscribers
    class PublicationTransport
    {
      public: PublicationTransport(const std::string &topic,
                                   const std::string &msgType);

      public: virtual ~PublicationTransport();

      public: void Init(const ConnectionPtr &conn);
      public: void Fini();
      public: void AddCallback(
                  const boost::function<void(const std::string &)> &cb);

      public: const ConnectionPtr GetConnection() const;
      public: std::string GetTopic() const;
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


