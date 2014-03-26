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
#ifndef _CONNECTION_MANAGER_HH_
#define _CONNECTION_MANAGER_HH_


#include <boost/shared_ptr.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <string>
#include <list>
#include <vector>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/SingletonT.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Connection.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class ConnectionManager ConnectionManager.hh transport/transport.hh
    /// \brief Manager of connections
    class GAZEBO_VISIBLE ConnectionManager :
      public SingletonT<ConnectionManager>
    {
      /// \brief Constructor
      private: ConnectionManager();

      /// \brief Destructor
      private: virtual ~ConnectionManager();

      /// \brief Initialize the connection manager.
      /// \param[in] _masterHost Host where the master is running.
      /// \param[in] _masterPort Port where the master is running.
      /// \param[in] _timeoutIterations Number of times to wait for
      /// a connection to master.
      /// \return true if initialization succeeded, false otherwise
      public: bool Init(const std::string &_masterHost,
                        unsigned int _masterPort,
                        uint32_t _timeoutIterations = 30);

      /// \brief Run the connection manager loop.  Does not return until
      /// stopped.
      public: void Run();

      /// \brief Is the manager running?
      /// \return true if running, false otherwise
      public: bool IsRunning() const;

      /// \brief Finalize the connection manager
      public: void Fini();

      /// \brief Stop the conneciton manager
      public: void Stop();

      /// \brief Subscribe to a topic.
      /// \param[in] _topic The topic to subscribe to.
      /// \param[in] _msgType The type of the topic.
      /// \param[in] _latching If true, latch the latest incoming message;
      /// otherwise don't.
      public: void Subscribe(const std::string &_topic,
                              const std::string &_msgType,
                              bool _latching);

      /// \brief Unsubscribe from a topic
      /// \param[in] _sub A subscription object
      public: void Unsubscribe(const msgs::Subscribe &_sub);

      /// \brief Unsubscribe from a topic
      /// \param[in] _topic The topic to unsubscribe from
      /// \param[in] _msgType The type of the topic
      public: void Unsubscribe(const std::string &_topic,
                                const std::string &_msgType);

      /// \brief Advertise a topic
      /// \param[in] _topic The topic to advertise
      /// \param[in] _msgType The type of the topic
      public: void Advertise(const std::string &_topic,
                              const std::string &_msgType);

      /// \brief Unadvertise a topic
      /// \param[in] _topic The topic to unadvertise
      public: void Unadvertise(const std::string &_topic);

      /// \brief Explicitly update the publisher list
      /// \param[out] _publishers The updated list of publishers is written here
      public: void GetAllPublishers(std::list<msgs::Publish> &_publishers);

      /// \brief Remove a connection from the manager
      /// \param[in] _conn The connection to be removed
      public: void RemoveConnection(ConnectionPtr &_conn);

      /// \brief Register a new topic namespace
      /// \param[in] _name The name of the topic namespace to be registered
      public: void RegisterTopicNamespace(const std::string &_name);

      /// \brief Get all the topic namespaces
      /// \param[out] _namespaces The list of namespace is written here
      public: void GetTopicNamespaces(std::list<std::string> &_namespaces);

      /// \brief Find a connection that matches a host and port
      /// \param[in] _host The host of the connection
      /// \param[in] _port The port of the connection
      /// \return Pointer to the connection; can be null (if no match was found)
      private: ConnectionPtr FindConnection(const std::string &_host,
                                            unsigned int _port);

      /// \brief Connect to a remote server
      /// \param[in] _host Host to connect to
      /// \param[in] _port Port to connect to
      /// \return Pointer to the connection; can be null (if connection failed)
      public: ConnectionPtr ConnectToRemoteHost(const std::string &_host,
                                                  unsigned int _port);

      /// \brief Inform the connection manager that it needs an update.
      public: void TriggerUpdate();

      /// \brief Callback function called when we have read data from the
      /// master
      /// \param[in] _data String of incoming data
      private: void OnMasterRead(const std::string &_data);

      /// \brief Callback function called when a connection is accepted
      /// \param[in] _newConnection Pointer to the new connection
      private: void OnAccept(ConnectionPtr _newConnection);

      /// \brief Callback function called when a connection is read
      /// \param[in] _newConnection Pointer to new connection
      /// \param[in] _data Data that has been read.
      private: void OnRead(ConnectionPtr _newConnection,
                           const std::string &_data);

      /// \brief Process a raw message.
      /// \param[in] _packet The raw message data.
      private: void ProcessMessage(const std::string &_packet);

      /// \brief Run the manager update loop once
      private: void RunUpdate();

      /// \brief Condition used to trigger an update.
      private: boost::condition_variable updateCondition;

      /// \brief Mutex for updateCondition
      private: boost::mutex updateMutex;

      private: ConnectionPtr masterConn;
      private: Connection *serverConn;

      private: std::list<ConnectionPtr> connections;
      protected: std::vector<event::ConnectionPtr> eventConnections;

      private: bool initialized;
      private: bool stop, stopped;

      private: unsigned int tmpIndex;
      private: boost::recursive_mutex listMutex;

      /// \brief A namespace to protect the namespace list.
      private: boost::mutex namespaceMutex;
      private: boost::recursive_mutex masterMessagesMutex;
      private: boost::recursive_mutex connectionMutex;

      private: std::list<msgs::Publish> publishers;
      private: std::list<std::string> namespaces;
      private: std::list<std::string> masterMessages;

      /// \brief Condition used for synchronization
      private: boost::condition_variable namespaceCondition;

      // Singleton implementation
      private: friend class SingletonT<ConnectionManager>;
    };
    /// \}
  }
}
#endif
