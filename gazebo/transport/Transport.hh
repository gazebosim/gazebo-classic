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
    /// \addtogroup gazebo_transport
    /// \{

    /// Get the hostname and port of the master from the GAZEBO_MASTER_URI
    /// environment variable
    /// \param master_host The hostname of the master is set to this param
    /// \param master_port The port of the master is set to this param
    /// \return False if the GAZEBO_MASTER_URI was not found
    bool get_master_uri(std::string &master_host, unsigned int &master_port);

    /// \brief Initialize the transport system
    /// \param master_host The hostname or IP of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    /// \param master_port The port  of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    bool init(const std::string &master_host ="",
              unsigned int master_port = 0);

    /// \brief Run the transport component. This starts message passing. This is
    ///        a blocking call
    void run();

    /// \brief Return all the namespace (world names) on the master
    void get_topic_namespaces(std::list<std::string> &_namespaces);

    /// \brief Return true if the transport system is stopped
    bool is_stopped();

    /// \brief Stop the transport component from running.
    void stop();

    /// \brief Cleanup the transport component
    void fini();

    /// \brief clear any remaining communication buffers
    void clear_buffers();

    /// \brief Set to true to pause incoming messages. They are still queued
    ///        for later delivery
    void pause_incoming(bool _pause);

    /// \brief Send a request, and receive a response.
    msgs::Response request(const std::string &_worldName,
                           const msgs::Request &_request);
    /// \}
  }
}
#endif
