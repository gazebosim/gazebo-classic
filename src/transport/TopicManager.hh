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

#include "transport/Publisher.hh"
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

                this->advertised_topics[topic]++;
                return PublisherPtr(new Publisher(topic, msg->GetTypeName()));
              }

      /// \brief Stop advertising on a topic
      public: void Unadvertise(const std::string &topic)
              {
                this->advertised_topics[topic]--;
              }

      /// \brief Send a message. Use a Publisher instead of calling this
      ///        function directly.
      /// \param topic Name of the topic
      /// \param message The message to send.
      public: void SendMessage( const std::string &topic, 
                                google::protobuf::Message &message );


      /// \brief Tells the topic manager about a publisher
      public: void ConnectSubscriber( const msgs::Publish &pub );

      private: void HandleIncoming();

      private: std::map<std::string, int> advertised_topics;
      private: std::map<std::string, std::list<CallbackHelperPtr> > subscribed_topics; 

      //Singleton implementation
      private: friend class DestroyerT<TopicManager>;
      private: friend class SingletonT<TopicManager>;
    };
  }
}
#endif

