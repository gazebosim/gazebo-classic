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

#ifndef PUBLICATION_HH
#define PUBLICATION_HH

#include <boost/shared_ptr.hpp>
#include <list>
#include <string>
#include <vector>

#include "transport/CallbackHelper.hh"
#include "transport/TransportTypes.hh"
#include "transport/PublicationTransport.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief A publication for a topic. This facilitates transport of
    /// messages
    class Publication
    {
      /// \brief Constructor
      public: Publication(const std::string &topic,
                           const std::string &msgType);

      /// \brief Destructor
      public: virtual ~Publication();

      /// \brief Get the type of message
      public: std::string GetMsgType() const;

      public: void AddSubscription(const CallbackHelperPtr &callback);
      public: void AddSubscription(const NodePtr &_node);

      /// \brief Remove a subscription
      public: void RemoveSubscription(const NodePtr &_node);

      /// \brief Remove a subscription
      public: void RemoveSubscription(const std::string &host,
                                      unsigned int port);

      public: void RemoveTransport(const std::string &host, unsigned int port);

      public: unsigned int GetTransportCount() const;
      public: unsigned int GetCallbackCount() const;
      public: unsigned int GetNodeCount() const;
      public: unsigned int GetRemoteSubscriptionCount();

      /// \brief Return true if the topic has been advertised from this
      ///        process.
      public: bool GetLocallyAdvertised() const;

      /// \brief Set whether this topic has been advertised from this process
      public: void SetLocallyAdvertised(bool _value);

      /// \brief Publish data
      public: void LocalPublish(const std::string &data);

      public: void Publish(const google::protobuf::Message &msg,
                           const boost::function<void()> &cb = NULL);

      public: void AddTransport(const PublicationTransportPtr &publink);
      public: bool HasTransport(const std::string &_host, unsigned int _port);

      public: void AddPublisher(PublisherPtr _pub);

      private: unsigned int id;
      private: static unsigned int idCounter;
      private: std::string topic;
      private: std::string msgType;

      /// \brief Remove nodes that receieve messages
      private: std::list<CallbackHelperPtr> callbacks;

      /// \brief Local nodes that recieve messages
      private: std::list<NodePtr> nodes;

      private: std::list<PublicationTransportPtr> transports;

      private: std::vector<PublisherPtr> publishers;

      private: bool locallyAdvertised;
    };
    /// \}
  }
}
#endif


