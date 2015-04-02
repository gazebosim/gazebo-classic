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

#ifndef _PUBLICATION_HH_
#define _PUBLICATION_HH_

#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <list>
#include <string>
#include <vector>
#include <map>

#include "gazebo/transport/CallbackHelper.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/PublicationTransport.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class Publication Publication.hh transport/transport.hh
    /// \brief A publication for a topic. This facilitates transport of
    /// messages
    class GAZEBO_VISIBLE Publication
    {
      /// \brief Constructor
      /// \param[in] _topic The topic we're publishing
      /// \param[in] _msgType The type of the topic we're publishing
      public: Publication(const std::string &_topic,
                           const std::string &_msgType);

      /// \brief Destructor
      public: virtual ~Publication();

      /// \brief Get the type of message
      /// \return The type of message
      public: std::string GetMsgType() const;

      /// \brief Subscribe a callback to our topic
      /// \param[in] _callback The callback
      public: void AddSubscription(const CallbackHelperPtr _callback);

      /// \brief Subscribe a node to our topic
      /// \param[in] _node The node
      public: void AddSubscription(const NodePtr &_node);

      /// \brief Unsubscribe a node from our topic
      /// \param[in] _node The node
      public: void RemoveSubscription(const NodePtr &_node);

      /// \brief Unsubscribe a a node by host/port from our topic
      /// \param[in] _host The node's hostname
      /// \param[in] _port The node's port
      public: void RemoveSubscription(const std::string &_host,
                                      unsigned int _port);

      /// \brief Remove a transport
      /// \param[in] _host The transport's hostname
      /// \param[in] _port The transport's port
      public: void RemoveTransport(const std::string &_host, unsigned
                                   int _port);

      /// \brief Get the number of transports
      /// \return The number of transports
      public: unsigned int GetTransportCount() const;

      /// \brief Get the number of callbacks
      /// \return The number of callbacks
      public: unsigned int GetCallbackCount() const;

      /// \brief Get the number of nodes
      /// \return The number of nodes
      public: unsigned int GetNodeCount() const;

      /// \brief Get the number of remote subscriptions
      /// \return The number of remote subscriptions
      public: unsigned int GetRemoteSubscriptionCount();

      /// \brief Was the topic has been advertised from this process?
      /// \return true if the topic has been advertised from this process,
      /// false otherwise
      public: bool GetLocallyAdvertised() const;

      /// \brief Set whether this topic has been advertised from this process
      /// \param[in] _value If true, the topic was locally advertise,
      /// otherwise it was not
      public: void SetLocallyAdvertised(bool _value);

      /// \brief Publish data to local subscribers (skip serialization)
      /// \param[in] _data The data to be published
      public: void LocalPublish(const std::string &_data);

      /// \brief Publish data to remote subscribers
      /// \param[in] _msg Message to be published
      /// \param[in] _cb Callback to be invoked after publishing
      /// is completed
      /// \return Number of remote subscribers that will receive the
      /// message.
      public: int Publish(MessagePtr _msg,
                  boost::function<void(uint32_t)> _cb,
                  uint32_t _id);

      /// \brief Remove a publisher.
      /// \param[in] _pub Pointer to publisher object to remove.
      public: void RemovePublisher(PublisherPtr _pub);

      /// \brief Set the previous message for a publisher.
      /// \param[in] _pubId ID of the publisher.
      /// \param[in] _msg The previous message.
      public: void SetPrevMsg(uint32_t _pubId, MessagePtr _msg);

      /// \brief Get a previous message for a publisher.
      /// \param[in] _pubId ID of the publisher.
      /// \return Pointer to the previous message. NULL if there is no
      /// previous message.
      public: MessagePtr GetPrevMsg(uint32_t _pubId);

      /// \brief Add a transport
      /// \param[in] _publink Pointer to publication transport object to
      /// be added
      public: void AddTransport(const PublicationTransportPtr &_publink);

      /// \brief Does a given transport exist?
      /// \param[in] _host Hostname of the transport
      /// \param[in] _port Port of the transport
      /// \return true if the transport exists, false otherwise
      public: bool HasTransport(const std::string &_host, unsigned int _port);

      /// \brief Add a publisher
      /// \param[in,out] _pub Pointer to publisher object to be added
      public: void AddPublisher(PublisherPtr _pub);

      /// \brief Remove nodes that have been marked for removal
      private: void RemoveNodes();

      /// \brief Unique if of the publication.
      private: unsigned int id;

      /// \brief Counter to produce unique IDs
      private: static unsigned int idCounter;

      /// \brief Name of the topic messages are output on.
      private: std::string topic;

      /// \brief Type of message produced through the publication
      private: std::string msgType;

      /// \brief Remote nodes that receieve messages.
      private: std::list<CallbackHelperPtr> callbacks;

      /// \brief Local nodes that recieve messages.
      private: std::list<NodePtr> nodes;

      /// \brief List of node IDs to remove from nodes list.
      private: std::list<unsigned int> removeNodes;

      /// \brief List of host and port callbacks to remove.
      private: std::list<std::pair<std::string, unsigned int> > removeCallbacks;

      /// \brief List of transport mechanisms.
      private: std::list<PublicationTransportPtr> transports;

      /// \brief List of publishers.
      private: std::vector<PublisherPtr> publishers;

      /// \brief True if the publication is advertised in the same process.
      private: bool locallyAdvertised;

      /// \brief Mutex to protect the list of nodes.
      private: mutable boost::mutex nodeMutex;

      /// \brief Mutex to protect the list of nodes.
      private: mutable boost::mutex callbackMutex;

      /// \brief Mutex to protect the list of nodes id for removed.
      private: mutable boost::mutex nodeRemoveMutex;

      /// \brief Publishers and their last messages.
      private: std::map<uint32_t, MessagePtr> prevMsgs;
    };
    /// \}
  }
}
#endif
