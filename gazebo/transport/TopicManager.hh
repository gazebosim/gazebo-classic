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
#ifndef TOPICMANAGER_HH
#define TOPICMANAGER_HH

#include <boost/bind.hpp>
#include <map>
#include <list>
#include <string>
#include <vector>

#include "common/Exception.hh"
#include "msgs/msgs.hh"
#include "common/SingletonT.hh"

#include "transport/TransportTypes.hh"
#include "transport/SubscribeOptions.hh"
#include "transport/SubscriptionTransport.hh"
#include "transport/PublicationTransport.hh"
#include "transport/ConnectionManager.hh"
#include "transport/Publisher.hh"
#include "transport/Publication.hh"
#include "transport/Subscriber.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief Manages topics and their subscriptions
    class TopicManager : public SingletonT<TopicManager>
    {
      private: TopicManager();
      private: virtual ~TopicManager();

      public: void Init();

      public: void Fini();

      public: PublicationPtr FindPublication(const std::string &topic);

      public: void AddNode(NodePtr _node);

      public: void RemoveNode(unsigned int _id);

      /// \param _onlyOut True means only outbound messages on nodes will be
      /// sent. False means nodes process both outbound and inbound messages
      public: void ProcessNodes(bool _onlyOut = false);

      /// \brief Returns true if the topic has been advertised
      /// \param _topic The name of the topic to check
      /// \return True if the topic has been advertised
      public: bool IsAdvertised(const std::string &_topic);

      /// \brief Subscribe to a topic
      public: SubscriberPtr Subscribe(const SubscribeOptions &options);

      /// \brief Unsubscribe from a topic. Use a Subscriber rather than
      ///        calling this function directly
      public: void Unsubscribe(const std::string &_topic, const NodePtr &_sub);

      /// \brief Advertise on a topic
      /// \param topic The name of the topic
      public: template<typename M>
              PublisherPtr Advertise(const std::string &_topic,
                                     unsigned int _queueLimit,
                                     bool _latch)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Advertise requires a google protobuf type");

                this->UpdatePublications(_topic, msg->GetTypeName());

                PublisherPtr pub = PublisherPtr(new Publisher(_topic,
                      msg->GetTypeName(), _queueLimit, _latch));

                std::string msgTypename;
                PublicationPtr publication;

                // Connect all local subscription to the publisher
                for (int i = 0; i < 2; i ++)
                {
                  std::string t;
                  if (i == 0)
                  {
                    t = _topic;
                    msgTypename = msg->GetTypeName();
                  }
                  else
                  {
                    t = _topic + "/__dbg";
                    msgs::GzString tmp;
                    msgTypename = tmp.GetTypeName();
                  }

                  publication = this->FindPublication(t);
                  publication->AddPublisher(pub);
                  if (!publication->GetLocallyAdvertised())
                  {
                    ConnectionManager::Instance()->Advertise(t, msgTypename);
                  }

                  publication->SetLocallyAdvertised(true);
                  pub->SetPublication(publication, i);

                  SubNodeMap::iterator iter2;
                  SubNodeMap::iterator st_end2 = this->subscribedNodes.end();
                  for (iter2 = this->subscribedNodes.begin();
                       iter2 != st_end2; iter2++)
                  {
                    if (iter2->first == t)
                    {
                      std::list<NodePtr>::iterator liter;
                      std::list<NodePtr>::iterator l_end = iter2->second.end();
                      for (liter = iter2->second.begin();
                           liter != l_end; liter++)
                      {
                        publication->AddSubscription(*liter);
                      }
                    }
                  }
                }

                return pub;
              }

      /// \brief Stop advertising on a topic
      public: void Unadvertise(const std::string &topic);

      /// \brief Send a message. Use a Publisher instead of calling this
      ///        function directly.
      /// \param topic Name of the topic
      /// \param message The message to send.
      /// \param cb Callback, used when the publish is completed.
      public: void Publish(const std::string &topic,
                            const google::protobuf::Message &message,
                            const boost::function<void()> &cb = NULL);

      /// \brief Connection a local Publisher to a remote Subscriber
      public: void ConnectPubToSub(const std::string &topic,
                                    const SubscriptionTransportPtr &sublink);

      /// \brief Connect a local Subscriber to a remote Publisher
      public: void ConnectSubToPub(const msgs::Publish &_pub);

      /// \brief Disconnect a local publisher from a remote subscriber
      public: void DisconnectPubFromSub(const std::string &topic,
                                         const std::string &host,
                                         unsigned int port);

      /// \brief Disconnection all local subscribers from a remote publisher
      public: void DisconnectSubFromPub(const std::string &topic,
                                         const std::string &host,
                                         unsigned int port);

      /// \brief Connect all subscribers on a topic to known publishers
      public: void ConnectSubscribers(const std::string &topic);

      /// \brief Update our list of advertised topics
      /// \return True if the provided params define a new publisher.
      public: PublicationPtr UpdatePublications(const std::string &topic,
                                                 const std::string &msgType);

      /// \brief Register a new topic namespace
      public: void RegisterTopicNamespace(const std::string &_name);

      /// \brief Get all the topic namespaces
      public: void GetTopicNamespaces(std::list<std::string> &_namespaces);

      public: void ClearBuffers();

      public: void PauseIncoming(bool _pause);

      typedef std::map<std::string, std::list<NodePtr> > SubNodeMap;

      private: typedef std::map<std::string, PublicationPtr> PublicationPtr_M;
      private: PublicationPtr_M advertisedTopics;
      private: PublicationPtr_M::iterator advertisedTopicsEnd;
      private: SubNodeMap subscribedNodes;
      private: std::vector<NodePtr> nodes;

      private: boost::recursive_mutex *nodeMutex;

      private: bool pauseIncoming;

      // Singleton implementation
      private: friend class SingletonT<TopicManager>;
    };
    /// \}
  }
}
#endif

