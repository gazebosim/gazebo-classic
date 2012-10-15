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
/* Desc: Handles pushing messages out on a named topic
 * Author: Nate Koenig
 */

#ifndef PUBLISHER_HH
#define PUBLISHER_HH

#include <google/protobuf/message.h>
#include <boost/thread.hpp>
#include <string>
#include <list>

#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief A publisher of messages on a topic
    class Publisher
    {
      /// \brief Use this constructor
      /// \param topic Name of topic
      /// \param msg_type Type of the message which is to be published
      public: Publisher(const std::string &topic, const std::string &msg_type,
                        unsigned int _limit, bool _latch);

      /// \brief Destructor
      public: virtual ~Publisher();

      public: bool HasConnections() const;

      /// \brief Block until a connection has been established with this
      ///        publisher
      public: void WaitForConnection() const;

      public: void SetPublication(PublicationPtr &_publication, int _i);

      /// \brief Publish a message on the topic
      public: void Publish(const google::protobuf::Message &_message,
                 bool _block = false)
              { this->PublishImpl(_message, _block); }
      public: template< typename M>
              void Publish(M _message, bool _block = false)
              { this->PublishImpl(_message, _block); }
      public: unsigned int GetOutgoingCount() const;

      private: void PublishImpl(const google::protobuf::Message &_message,
                                bool _block);

      /// \brief Get the topic name
      public: std::string GetTopic() const;

      /// \brief Get the message type
      public: std::string GetMsgType() const;

      /// \brief Send latest message over the wire. For internal use only
      public: void SendMessage();
      public: bool GetLatching() const;

      public: std::string GetPrevMsg() const;

      /// \brief Callback when a publish is completed
      private: void OnPublishComplete();

      private: std::string topic;
      private: std::string msgType;
      private: unsigned int queueLimit;
      private: std::list<google::protobuf::Message *> messages;
      private: boost::recursive_mutex *mutex;
      private: PublicationPtr publications[2];

      private: bool latch;
      private: google::protobuf::Message *prevMsg;
    };
    /// \}
  }
}

#endif


