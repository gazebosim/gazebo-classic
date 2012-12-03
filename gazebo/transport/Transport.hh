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
#ifndef _GAZEBO_TRANSPORT_HH_
#define _GAZEBO_TRANSPORT_HH_

#include <boost/bind.hpp>
#include <string>
#include <list>

#include "transport/TransportTypes.hh"
#include "transport/SubscribeOptions.hh"
#include "transport/TopicManager.hh"

namespace gazebo
{
  namespace transport
  {
    /// \brief Handles transportation of messages
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief Get the hostname and port of the master from the
    /// GAZEBO_MASTER_URI environment variable
    /// \param[out] _master_host The hostname of the master is set to this param
    /// \param[out] _master_port The port of the master is set to this param
    /// \return true if GAZEBO_MASTER_URI was successfully parsed; false
    /// otherwise (in which case output params are not set)
    bool get_master_uri(std::string &_master_host, unsigned int &_master_port);

    /// \brief Initialize the transport system
    /// \param[in] _master_host The hostname or IP of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    /// \param[in] _master_port The port  of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    /// \return true if initialization succeeded; false otherwise
    bool init(const std::string &_master_host ="",
              unsigned int _master_port = 0);

    /// \brief Run the transport component. Creates a thread to handle
    /// message passing. This call will block until the master can
    /// be contacted or until a retry limit is reached
    void run();

    /// \brief Return all the namespace (world names) on the master
    /// \param[out] _namespaces The list of namespace will be written here
    void get_topic_namespaces(std::list<std::string> &_namespaces);

    /// \brief Is the transport system stopped?
    /// \return true if the transport system is stopped; false otherwise
    bool is_stopped();

    /// \brief Stop the transport component from running.
    void stop();

    /// \brief Cleanup the transport component
    void fini();

    /// \brief Clear any remaining communication buffers
    void clear_buffers();

    /// \brief Pause or unpause incoming messages. When paused, messages
    /// are queued for later delivery
    /// \param[in] _pause If true, pause; otherwise unpause
    void pause_incoming(bool _pause);

    /// \brief Send a request and receive a response.  This call will block
    /// until a response is received.
    /// \param[in] _worldName The name of the world to which the request
    /// should be sent
    /// \param[in] _request The request itself
    /// \return The response to the request.  Can be empty.
    msgs::Response request(const std::string &_worldName,
                           const msgs::Request &_request);
    /// \}
  }
}
#endif
