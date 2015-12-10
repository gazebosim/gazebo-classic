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
/* Desc: Handles pushing messages out on a named topic
 * Author: Nate Koenig
 */

#ifndef _PUBLISHER_HH_
#define _PUBLISHER_HH_

#include <google/protobuf/message.h>
#include <boost/thread.hpp>
#include <string>
#include <list>
#include <map>

#include "gazebo/common/Time.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class Publisher Publisher.hh transport/transport.hh
    /// \brief A publisher of messages on a topic
    class GZ_TRANSPORT_VISIBLE Publisher
      : public boost::enable_shared_from_this<Publisher>
    {
      /// \brief Constructor
      /// \param[in] _topic Name of topic to be published
      /// \param[in] _msgType Type of the message to be published
      /// \param[in] _limit Maximum number of outgoing messages to queue
      /// \param[in] _hz Update rate for the publisher. Units are
      /// 1.0/seconds.
      public: Publisher(const std::string &_topic, const std::string &_msgType,
                        unsigned int _limit, double _hzRate);

      /// \brief Destructor
      public: virtual ~Publisher();

      /// \brief Are there any connections?
      /// \return true if there are any connections, false otherwise
      public: bool HasConnections() const;

      /// \brief Block until a connection has been established with this
      ///        publisher
      public: void WaitForConnection() const;

      /// \brief Block until a connection has been established with this
      ///        publisher
      /// \param[in] _timeout Maxiumum time to wait. Use a negative time
      /// value to wait forever.
      /// \return True if a connection was established.
      public: bool WaitForConnection(const common::Time &_timeout) const;

      /// \brief Set the publication object for a particular publication
      /// \param[in] _publication Pointer to the publication object to be set
      public: void SetPublication(PublicationPtr _publication);

      /// \brief Publish a protobuf message on the topic
      /// \param[in] _message Message to be published
      /// \param[in] _block Whether to block until the message is actually
      /// written out
      public: void Publish(const google::protobuf::Message &_message,
                 bool _block = false)
              { this->PublishImpl(_message, _block); }

      /// \brief Publish an arbitrary message on the topic
      /// \param[in] _message Message to be published
      /// \param[in] _block Whether to block until the message is actually
      /// written out
      public: template< typename M>
              void Publish(M _message, bool _block = false)
              { this->PublishImpl(_message, _block); }

      /// \brief Get the number of outgoing messages
      /// \return The number of outgoing messages
      public: unsigned int GetOutgoingCount() const;

      /// \brief Get the topic name
      /// \return The topic name
      public: std::string GetTopic() const;

      /// \brief Get the message type
      /// \return The message type
      public: std::string GetMsgType() const;

      /// \brief Send latest message over the wire. For internal use only
      public: void SendMessage();

      /// \brief Set our containing node.
      /// \param[in] _node Pointer to a node. Should be the node that create
      /// this publisher.
      public: void SetNode(NodePtr _node);

      /// \brief Get the previously published message
      /// \return The previously published message, if any
      public: std::string GetPrevMsg() const;

      /// \brief Get the previously published message
      /// \return The previously published message, if any
      public: MessagePtr GetPrevMsgPtr() const;

      /// \brief Finalize the publisher.
      public: void Fini();

      /// \brief Get the id of this publisher.
      /// \return Unique id of this publisher.
      public: uint32_t Id() const;

      /// \brief Implementation of Publish.
      /// \param[in] _message Message to be published.
      /// \param[in] _block Whether to block until the message is actually
      /// written out.
      private: void PublishImpl(const google::protobuf::Message &_message,
                                bool _block);

      /// \brief Callback when a publish is completed
      /// \param[in] _id ID associated with the publication.
      private: void OnPublishComplete(uint32_t _id);

      /// \brief Topic on which messages are published.
      private: std::string topic;

      /// \brief Type of message published.
      private: std::string msgType;

      /// \brief Maximum number of messages that can be queued prior to
      /// publication.
      private: unsigned int queueLimit;

      /// \brief Period at which messages are published. Zero indicates no
      /// limit.
      private: double updatePeriod;

      /// \brief True if queueLimit has been reached, and a warning message
      /// was produced.
      private: bool queueLimitWarned;

      /// \brief List of messages to publish.
      private: std::list<MessagePtr> messages;

      /// \brief For mutual exclusion.
      private: mutable boost::mutex mutex;

      /// \brief The publication pointers. One for normal publication, and
      /// one for debug.
      private: PublicationPtr publication;

      /// \brief Pointer to our containing node.
      private: NodePtr node;

      private: common::Time currentTime;
      private: common::Time prevPublishTime;

      /// \brief Current id of the sent message.
      private: uint32_t pubId;

      /// \brief Current publication ids.
      private: std::map<uint32_t, int> pubIds;

      /// \brief Unique ID for this publisher.
      private: uint32_t id;

      /// \brief Counter to create unique ID for publishers.
      private: static uint32_t idCounter;
    };
    /// \}
  }
}
#endif
