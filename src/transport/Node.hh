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

#ifndef NODE_HH
#define NODE_HH

#include "transport/TransportTypes.hh"
#include "transport/TopicManager.hh"

namespace gazebo
{
  namespace transport
  {
    class Node
    {
      public: Node();
      public: virtual ~Node();

      /// \brief Init the node
      /// \param space Set the global namespace of all topics
      public: void Init(const std::string &space);

      /// \brief Decode a topic name
      public: std::string DecodeTopicName(const std::string &topic);

      /// \brief Encode a topic name
      public: std::string EncodeTopicName(const std::string &topic);

      /// \brief Adverise a topic
      template<typename M>
      transport::PublisherPtr Advertise(const std::string &topic)
      {
        std::string decodedTopic = this->DecodeTopicName(topic);

        return transport::TopicManager::Instance()->Advertise<M>(decodedTopic); 
      }

      /// \brief Subscribe to a topic, and return data on the callback
      template<typename M, typename T>
      SubscriberPtr Subscribe(const std::string &topic,
          void(T::*fp)(const boost::shared_ptr<M const> &), T *obj)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(topic);
        ops.template Init<M>(decodedTopic, boost::bind(fp, obj, _1));
        return transport::TopicManager::Instance()->Subscribe(ops);
      }
  
      /// \brief Subscribe to a topic, and return data on the callback
      template<typename M>
      SubscriberPtr Subscribe(const std::string &topic,
          void(*fp)(const boost::shared_ptr<M const> &))
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(topic);
        ops.template Init<M>(decodedTopic, fp);
        return transport::TopicManager::Instance()->Subscribe(ops);
      }

      private: std::string topicNamespace;
    };
  }
}
#endif
