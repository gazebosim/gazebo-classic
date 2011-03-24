/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include <map>
#include <list>
#include <boost/bind.hpp>

#include "common/GazeboError.hh"
#include "common/Messages.hh"
#include "common/SingletonT.hh"

#include "transport/Connection.hh"
#include "transport/Publisher.hh"
#include "transport/Publication.hh"
#include "transport/Subscriber.hh"

namespace gazebo
{
  namespace transport
  {
    /// \brief Manages topics and their subscriptions
    class TopicManager : public SingletonT<TopicManager>
    {
      private: TopicManager();
      private: virtual ~TopicManager();

      public: void Init(unsigned short port);

      public: PublicationPtr FindPublication(const std::string &topic);

      /// \brief Subscribe to a topic
      public: template<class M, class T>
              SubscriberPtr Subscribe(const std::string &topic, void(T::*fp)(const boost::shared_ptr<M const> &), T *obj)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Subscribe requires a google protobuf type");

                CallbackHelperPtr subscription( 
                    new CallbackHelperT<M>( boost::bind(fp, obj, _1) ) );

                SubscriberPtr sub( new Subscriber(topic, subscription) );

                std::cout << "TopicManager::Subscribe to topic[" << topic << "]\n";
                this->subscribed_topics[topic].push_back(subscription);

                return sub;
              }

        /*public: template<class M>
                Subscriber Subscribe(const std::string &topic, const boost::function<void (const boost::shared_ptr<M const>&)> &callback)
                {
                }
                */

      /// \brief Unsubscribe from a topic. Use a Subscriber rather than
      ///        calling this function directly
      public: void Unsubscribe(const std::string &topic, CallbackHelperPtr sub);

      /// \brief Advertise on a topic
      /// \param topic The name of the topic
      public: template<typename M>
              PublisherPtr Advertise(const std::string topic)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Advertise requires a google protobuf type");

                this->UpdatePublications(topic, msg->GetTypeName());

                return PublisherPtr( new Publisher(topic, msg->GetTypeName()) );
              }

      /// \brief Stop advertising on a topic
      public: void Unadvertise(const std::string &topic)
              {
                // TODO: Implement this
              }

      /// \brief Send a message. Use a Publisher instead of calling this
      ///        function directly.
      /// \param topic Name of the topic
      /// \param message The message to send.
      public: void Publish( const std::string &topic, 
                            google::protobuf::Message &message );

      /// \brief Connection a local Publisher to a remote Subscriber
      public: void ConnectPubToSub( const msgs::Subscribe &sub,
                                    const ConnectionPtr &connection );

      /// \brief Connect a local Subscriber to a remote Publisher
      public: void ConnectSubToPub( const msgs::Publish &pub,
                                    const ConnectionPtr &connection );

      public: void UpdatePublications( const std::string &topic, 
                                       const std::string &msgType );

      private: void HandleIncoming();

      private: std::list<PublicationPtr> advertisedTopics;
      //private: std::list<Subscription> subscriptions;

      //private: std::map<std::string, int> advertised_topics;
      private: std::map<std::string, std::list<CallbackHelperPtr> > subscribed_topics; 

      //private: std::map<std::string, ConnectionPtr> subscribedConnections;

      //Singleton implementation
      private: friend class DestroyerT<TopicManager>;
      private: friend class SingletonT<TopicManager>;
    };
  }
}
#endif

