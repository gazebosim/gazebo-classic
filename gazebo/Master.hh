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
#ifndef MASTER_HH
#define MASTER_HH

#include <string>
#include <list>
#include <deque>
#include <utility>
#include <map>
#include <boost/shared_ptr.hpp>

#include "msgs/msgs.hh"
#include "transport/Connection.hh"

namespace gazebo
{
  /// \brief A ROS Master-like manager that directs gztopic connections, enables
  ///        each gazebo network client to locate one another for peer-to-peer
  ///        communication.
  class Master
  {
    /// \brief Constructor
    public: Master();

    /// \brief Destructor
    public: virtual ~Master();

    /// \brief Initialize
    /// \param _port The master's port
    public: void Init(uint16_t _port);

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
    /// \param _connectionIndex Index of the connection which generated the
    ///                         message
    /// \param _data The message data
    private: void ProcessMessage(const unsigned int _connectionIndex,
                                 const std::string &_data);

    /// \brief Connection read callback
    /// \param _connectionIndex Index of the connection which generated the
    ///                         message
    /// \param _data The message data
    private: void OnRead(const unsigned int _connectionIndex,
                         const std::string &_data);

    /// \brief Accept a new connection
    /// \param _newConnection The new connection
    private: void OnAccept(const transport::ConnectionPtr &_newConnection);

    /// \brief Get a publisher for the given topic
    /// \param _topic Name of the topic
    /// \return A publish message
    private: msgs::Publish GetPublisher(const std::string &_topic);

    /// \brief Find a connection given a host and port
    /// \param _host Host name
    /// \param _port Port number
    /// \return The found connection, or NULL
    private: transport::ConnectionPtr FindConnection(const std::string &_host,
                                                     uint16_t _port);


    private: void RemoveConnection(unsigned int _index);
    private: void RemovePublisher(const msgs::Publish _pub);
    private: void RemoveSubscriber(const msgs::Subscribe _sub);

    typedef std::map<unsigned int, transport::ConnectionPtr> Connection_M;
    typedef std::list< std::pair<msgs::Publish, transport::ConnectionPtr> >
      PubList;
    typedef std::list< std::pair<msgs::Subscribe, transport::ConnectionPtr> >
      SubList;
    private: PubList publishers;
    private: SubList subscribers;

    private: Connection_M connections;
    private: std::list< std::string > worldNames;
    private: std::list< std::pair<unsigned int, std::string> > msgs;

    private: transport::Connection *connection;
    private: boost::thread *runThread;
    private: bool stop;

    private: boost::recursive_mutex *connectionMutex, *msgsMutex;
  };
}

#endif
