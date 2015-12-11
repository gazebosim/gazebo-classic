/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _MASTER_HH_
#define _MASTER_HH_

#include <string>
#include <list>
#include <deque>
#include <utility>
#include <map>
#include <boost/shared_ptr.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Connection.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  // Forward declare private data class
  class MasterPrivate;

  /// \class Master Master.hh gazebo_core.hh
  /// \brief A manager that directs topic connections, enables each gazebo
  /// network client to locate one another for peer-to-peer communication.
  class GAZEBO_VISIBLE Master
  {
    public:
      /// \def Map of unique id's to connections.
      typedef std::map<unsigned int, transport::ConnectionPtr> Connection_M;

    public:
      /// \def Map of publish messages to connections.
      typedef std::list< std::pair<msgs::Publish, transport::ConnectionPtr> >
        PubList;

    public:
      /// \def Map of subscribe messages to connections.
      typedef std::list< std::pair<msgs::Subscribe, transport::ConnectionPtr> >
        SubList;

    /// \brief Constructor
    public: Master();

    /// \brief Destructor
    public: virtual ~Master();

    /// \brief Initialize
    /// \param[in] _port The master's port
    public: void Init(uint16_t _port);

    /// \brief Run the master.
    public: void Run();

    /// \brief Run the master in a new thread
    public: void RunThread();

    /// \brief Run the master one iteration
    public: void RunOnce();

    /// \brief Stop the master
    public: void Stop();

    /// \brief Finalize the master
    public: void Fini();

    /// \brief Process a message
    /// \param[in] _connectionIndex Index of the connection which generated the
    /// message
    /// \param[i] _data The message data
    private: void ProcessMessage(const unsigned int _connectionIndex,
                                 const std::string &_data);

    /// \brief Connection read callback
    /// \param[in] _connectionIndex Index of the connection which generated the
    ///  message
    /// \param[in] _data The message data
    private: void OnRead(const unsigned int _connectionIndex,
                         const std::string &_data);

    /// \brief Accept a new connection
    /// \param[in] _newConnection The new connection
    private: void OnAccept(transport::ConnectionPtr _newConnection);

    /// \brief Get a publisher for the given topic
    /// \param[in] _topic Name of the topic
    /// \return A publish message
    private: msgs::Publish GetPublisher(const std::string &_topic);

    /// \brief Find a connection given a host and port
    /// \param[in] _host Host name
    /// \param[in] _port Port number
    /// \return The found connection, or NULL
    private: transport::ConnectionPtr FindConnection(const std::string &_host,
                                                     uint16_t _port);

    /// \brief Remove a connection.
    /// \param[in] _connIter Iterator to the connection to remove.
    /// _connIter will be incremented when removed.
    private: void RemoveConnection(Connection_M::iterator _connIter);

    /// \brief Remove a publisher.
    /// \param[in] _pub Publish message that contains the info necessary to
    /// remove a publisher.
    private: void RemovePublisher(const msgs::Publish _pub);

    /// \brief Remove a subscriber.
    /// \param[in] _pub Subscribe message that contains the info necessary to
    /// remove a subscriber.
    private: void RemoveSubscriber(const msgs::Subscribe _sub);

    /// \internal
    /// \brief Pointer to private data.
    protected: std::unique_ptr<MasterPrivate> dataPtr;
  };
}
#endif
