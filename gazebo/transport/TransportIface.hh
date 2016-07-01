/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_TRANSPORTIFACE_HH_
#define _GAZEBO_TRANSPORTIFACE_HH_

#include <boost/bind.hpp>
#include <string>
#include <list>
#include <map>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/SubscribeOptions.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TopicManager.hh"

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
    GZ_TRANSPORT_VISIBLE
    bool get_master_uri(std::string &_master_host, unsigned int &_master_port);

    /// \brief Initialize the transport system
    /// \param[in] _masterHost The hostname or IP of the master. Leave empty to
    /// use pull address from the GAZEBO_MASTER_URI env var.
    /// \param[in] _masterPort The port  of the master. Leave empty to
    /// use pull address from the GAZEBO_MASTER_URI env var.
    /// \param[in] _timeoutIterations Number of times to wait for
    /// a connection to master.
    /// \return true if initialization succeeded; false otherwise
    GZ_TRANSPORT_VISIBLE
    bool init(const std::string &_masterHost = "",
              unsigned int _masterPort = 0,
              uint32_t _timeoutIterations = 30);

    /// \brief Run the transport component. Creates a thread to handle
    /// message passing. This call will block until the master can
    /// be contacted or until a retry limit is reached
    GZ_TRANSPORT_VISIBLE
    void run();

    /// \brief Return all the namespace (world names) on the master
    /// \param[out] _namespaces The list of namespace will be written here
    GZ_TRANSPORT_VISIBLE
    void get_topic_namespaces(std::list<std::string> &_namespaces);

    /// \brief Is the transport system stopped?
    /// \return true if the transport system is stopped; false otherwise
    GZ_TRANSPORT_VISIBLE
    bool is_stopped();

    /// \brief Stop the transport component from running.
    GZ_TRANSPORT_VISIBLE
    void stop();

    /// \brief Cleanup the transport component
    GZ_TRANSPORT_VISIBLE
    void fini();

    /// \brief Clear any remaining communication buffers
    GZ_TRANSPORT_VISIBLE
    void clear_buffers();

    /// \brief Pause or unpause incoming messages. When paused, messages
    /// are queued for later delivery
    /// \param[in] _pause If true, pause; otherwise unpause
    GZ_TRANSPORT_VISIBLE
    void pause_incoming(bool _pause);

    /// \brief Send a request and receive a response.  This call will block
    /// until a response is received.
    /// \param[in] _worldName The name of the world to which the request
    /// should be sent
    /// \param[in] _request The type request.
    /// \param[in] _data Optional data string.
    /// \return The response to the request.  Can be empty.
    GZ_TRANSPORT_VISIBLE
    boost::shared_ptr<msgs::Response> request(const std::string &_worldName,
                                              const std::string &_request,
                                              const std::string &_data = "");

    /// \brief Send a request and don't wait for a response. This is
    /// non-blocking.
    /// \param[in] _worldName The name of the world to which the request
    /// should be sent.
    /// \param[in] _request The type request.
    /// \param[in] _data Optional data string.
    GZ_TRANSPORT_VISIBLE
    void requestNoReply(const std::string &_worldName,
                        const std::string &_request,
                        const std::string &_data = "");

    /// \brief Send a request and don't wait for a response. This is
    /// non-blocking.
    /// \param[in] _node Pointer to a node that provides communication.
    /// \param[in] _request The type request.
    /// \param[in] _data Optional data string.
    GZ_TRANSPORT_VISIBLE
    void requestNoReply(NodePtr _node, const std::string &_request,
                        const std::string &_data = "");

    /// \brief A convenience function for a one-time publication of
    /// a message. This is inefficient, compared to
    /// Node::Advertise followed by Publisher::Publish. This function
    /// should only be used when sending a message very infrequently.
    /// \param[in] _topic The topic to advertise
    /// \param[in] _message Message to be published
    template<typename M>
    void publish(const std::string &_topic,
                 const google::protobuf::Message &_message)
    {
      transport::NodePtr node = transport::NodePtr(new transport::Node());
      node->Init();
      node->Publish<M>(_topic, _message);
    }

    /// \brief Get a list of all the topics and their message types.
    /// \return A map where keys are message types, and values are a list
    /// of topic names.
    GZ_TRANSPORT_VISIBLE
    std::map<std::string, std::list<std::string> > getAdvertisedTopics();

    /// \brief Get a list of all the unique advertised topic names.
    /// \param[in] _msgType Type of message to filter the result on. If
    /// empty, then a list of all the topics is returned.
    /// \return A list of the advertised topics that publish messages
    /// of the type specified by _msgType.
    GZ_TRANSPORT_VISIBLE
    std::list<std::string> getAdvertisedTopics(const std::string &_msgType);

    /// \brief Get the message typename that is published on the given topic.
    /// \param[in] _topicName Name of the topic to query.
    /// \return The message type, or empty string if the topic is not valid.
    GZ_TRANSPORT_VISIBLE
    std::string getTopicMsgType(const std::string &_topicName);

    /// \brief Set whether minimal comms should be used. This will be used
    /// to reduce network traffic.
    GZ_TRANSPORT_VISIBLE
    void setMinimalComms(bool _enabled);

    /// \brief Get whether minimal comms has been enabled.
    /// \return True if minimal comms is enabled.
    GZ_TRANSPORT_VISIBLE
    bool getMinimalComms();

    /// \brief Create a connection to master.
    /// \return Connection to the master, NULL on error.
    GZ_TRANSPORT_VISIBLE
    transport::ConnectionPtr connectToMaster();

    /// \brief Blocks while waiting for topic namespaces from the Master.
    /// This function will wait a maximum of _maxWait.
    /// \return True if namespaces were found before _maxWait time.
    GZ_TRANSPORT_VISIBLE
    bool waitForNamespaces(const gazebo::common::Time &_maxWait);
    /// \}
  }
}
#endif
