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
#ifndef _TOPICMANAGER_HH_
#define _TOPICMANAGER_HH_

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

    /// \class TopicManager TopicManager.hh transport/transport.hh
    /// \brief Manages topics and their subscriptions
    class TopicManager : public SingletonT<TopicManager>
    {
      private: TopicManager();
      private: virtual ~TopicManager();

      /// \brief Initialize the manager
      public: void Init();

      /// \brief Finalize the manager
      public: void Fini();

      /// \brief Find a publication object by topic
      /// \param[in] _topic The topic to search for
      /// \return Pointer to the publication object, if found (can be null)
      public: PublicationPtr FindPublication(const std::string &_topic);

      /// \brief Add a node to the manager
      /// \param[in,out] _node The node to be added
      public: void AddNode(NodePtr _node);

      /// \brief Remove a node by its id
      /// \param[in] _id The ID of the node to be removed
      public: void RemoveNode(unsigned int _id);

      /// \brief Process all nodes under management
      /// \param[in] _onlyOut True means only outbound messages on nodes will be
      /// sent. False means nodes process both outbound and inbound messages
      public: void ProcessNodes(bool _onlyOut = false);

      /// \brief Has the topic been advertised?
      /// \param[in] _topic The name of the topic to check
      /// \return true if the topic has been advertised, false otherwise
      public: bool IsAdvertised(const std::string &_topic);

      /// \brief Subscribe to a topic
      /// \param[in] _options The options to use for the subscription
      /// \return Pointer to the newly created subscriber
      public: SubscriberPtr Subscribe(const SubscribeOptions &_options);

      /// \brief Unsubscribe from a topic. Use a Subscriber rather than
      ///        calling this function directly
      /// \param[in] _topic The topic to unsubscribe from
      /// \param[in] _sub The node to unsubscribe
      public: void Unsubscribe(const std::string &_topic, const NodePtr &_sub);

      /// \brief Advertise on a topic
      /// \param[in] _topic The name of the topic
      /// \param[in] _queueLimit The maximum number of outgoing messages
      /// to queue
      /// \return Pointer to the newly created Publisher
      public: template<typename M>
              PublisherPtr Advertise(const std::string &_topic,
                                     unsigned int _queueLimit)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Advertise requires a google protobuf type");

                this->UpdatePublications(_topic, msg->GetTypeName());

                PublisherPtr pub = PublisherPtr(new Publisher(_topic,
                      msg->GetTypeName(), _queueLimit));

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

      /// \brief Unadvertise a topic
      /// \param[in] _topic The topic to be unadvertised
      public: void Unadvertise(const std::string &_topic);

      /// \brief Send a message. Use a Publisher instead of calling this
      ///        function directly.
      /// \param _topic Name of the topic
      /// \param _message The message to send.
      /// \param _cb Callback, used when the publish is completed.
      public: void Publish(const std::string &_topic,
                            const google::protobuf::Message &_message,
                            const boost::function<void()> &_cb = NULL);

      /// \brief Connection a local Publisher to a remote Subscriber
      /// \param[in] _topic The topic to use
      /// \param[in] _sublink The subscription transport object to use
      public: void ConnectPubToSub(const std::string &_topic,
                                    const SubscriptionTransportPtr &_sublink);

      /// \brief Connect a local Subscriber to a remote Publisher
      /// \param[in] _pub The publish object to use
      public: void ConnectSubToPub(const msgs::Publish &_pub);

      /// \brief Disconnect a local publisher from a remote subscriber
      /// \param[in] _topic The topic to be disconnected
      /// \param[in] _host The host to be disconnected
      /// \param[in] _port The port to be disconnected
      public: void DisconnectPubFromSub(const std::string &_topic,
                                         const std::string &_host,
                                         unsigned int _port);

      /// \brief Disconnect all local subscribers from a remote publisher
      /// \param[in] _topic The topic to be disconnected
      /// \param[in] _host The host to be disconnected
      /// \param[in] _port The port to be disconnected
      public: void DisconnectSubFromPub(const std::string &_topic,
                                         const std::string &_host,
                                         unsigned int _port);

      /// \brief Connect all subscribers on a topic to known publishers
      /// \param[in] _topic The topic to be connected
      public: void ConnectSubscribers(const std::string &_topic);

      /// \brief Update our list of advertised topics
      /// \param[in] _topic The topic to be updated
      /// \param[in] _msgType The type of the topic to be updated
      /// \return True if the provided params define a new publisher,
      /// false otherwise
      public: PublicationPtr UpdatePublications(const std::string &_topic,
                                                const std::string &_msgType);

      /// \brief Register a new topic namespace
      /// \param[in] _name The name of the new namespace
      public: void RegisterTopicNamespace(const std::string &_name);

      /// \brief Get all the topic namespaces
      /// \param[out] _namespaces The list of namespaces will be written here
      public: void GetTopicNamespaces(std::list<std::string> &_namespaces);

      /// \brief Get a list of all the topics.
      /// \return A map where keys are message types, and values are a list
      /// of topic names.
      /// \sa transport::GetAdvertisedTopics
      public: std::map<std::string, std::list<std::string> >
              GetAdvertisedTopics() const GAZEBO_DEPRECATED;

      /// \brief Clear all buffers
      public: void ClearBuffers();

      /// \brief Pause or unpause processing of incoming messages
      /// \param[in] _pause If true pause processing; otherwse unpause
      public: void PauseIncoming(bool _pause);

      /// \brief A map of string->list of Node pointers
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
