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
#ifndef CONNECTION_MANAGER_HH
#define CONNECTION_MANAGER_HH


#include <boost/shared_ptr.hpp>
#include <string>
#include <list>
#include <vector>

#include "msgs/msgs.hh"
#include "common/SingletonT.hh"

#include "transport/Publisher.hh"
#include "transport/Connection.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief Manager of connections
    class ConnectionManager : public SingletonT<ConnectionManager>
    {
      /// \brief Constructor
      private: ConnectionManager();

      /// \brief Destructor
      private: virtual ~ConnectionManager();

      public: bool Init(const std::string &master_host,
                        unsigned int master_port);

      /// \brief Run the connection manager loop
      public: void Run();

      /// \brief Return true if running (not stopped)
      public: bool IsRunning() const;

      /// \brief Finalize the conneciton manager
      public: void Fini();

      /// \brief Stop the conneciton manager
      public: void Stop();

      public: void Subscribe(const std::string &_topic,
                              const std::string &_msgType,
                              bool _latching);

      public: void Unsubscribe(const msgs::Subscribe &_sub);

      public: void Unsubscribe(const std::string &_topic,
                                const std::string &_msgType);

      public: void Advertise(const std::string &topic,
                              const std::string &msgType);

      public: void Unadvertise(const std::string &topic);

      /// \brief Explicitly update the publisher list
      public: void GetAllPublishers(std::list<msgs::Publish> &publishers);

      /// \brief Remove a connection
      public: void RemoveConnection(ConnectionPtr &conn);

      /// \brief Register a new topic namespace
      public: void RegisterTopicNamespace(const std::string &_name);

      /// \brief Get all the topic namespaces
      public: void GetTopicNamespaces(std::list<std::string> &_namespaces);

      /// \brief Find a connection that matches a host and port
      private: ConnectionPtr FindConnection(const std::string &host,
                                            unsigned int port);

      /// \brief Connect to a remote server
      public: ConnectionPtr ConnectToRemoteHost(const std::string &host,
                                                  unsigned int port);

      private: void OnMasterRead(const std::string &data);

      private: void OnAccept(const ConnectionPtr &new_connection);

      private: void OnRead(const ConnectionPtr &new_connection,
                            const std::string &data);

      private: void ProcessMessage(const std::string &_packet);

      public: void RunUpdate();

      private: ConnectionPtr masterConn;
      private: Connection *serverConn;

      private: std::list<ConnectionPtr> connections;
      protected: std::vector<event::ConnectionPtr> eventConnections;

      private: bool initialized;
      private: bool stop, stopped;
      private: boost::thread *thread;

      private: unsigned int tmpIndex;
      private: boost::recursive_mutex *listMutex;
      private: boost::recursive_mutex *masterMessagesMutex;
      private: boost::recursive_mutex *connectionMutex;

      private: std::list<msgs::Publish> publishers;
      private: std::list<std::string> namespaces;
      private: std::list<std::string> masterMessages;

      // Singleton implementation
      private: friend class SingletonT<ConnectionManager>;
    };
    /// \}
  }
}

#endif


