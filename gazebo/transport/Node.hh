/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_TRANSPORT_NODE_HH_
#define GAZEBO_TRANSPORT_NODE_HH_

#undef emit
#include <tbb/task.h>
#define emit
#ifndef TBB_VERSION_MAJOR
#include <tbb/version.h>
#endif

// This fixes compiler warnings, see #3147 and #3160
#ifndef BOOST_BIND_GLOBAL_PLACEHOLDERS
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif

#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <map>
#include <list>
#include <string>
#include <vector>
#if TBB_VERSION_MAJOR >= 2021
#include "gazebo/transport/TaskGroup.hh"
#endif
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/TopicManager.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \cond
    /// \brief Task used by Node::Publish to publish on a one-time publisher
#if TBB_VERSION_MAJOR < 2021
    class GZ_TRANSPORT_VISIBLE PublishTask : public tbb::task
#else
    class GZ_TRANSPORT_VISIBLE PublishTask
#endif
    {
      /// \brief Constructor
      /// \param[in] _pub Publisher to publish the message on.
      /// \param[in] _message Message to publish
      public: PublishTask(transport::PublisherPtr _pub,
                  const google::protobuf::Message &_message)
              : pub(_pub)
      {
        this->msg = _message.New();
        this->msg->CopyFrom(_message);
      }

#if TBB_VERSION_MAJOR < 2021
      /// \brief Overridden function from tbb::task that exectues the
      /// publish task.
      public: tbb::task *execute()
#else
      /// \brief Executes the publish task.
      public: void operator()() const
#endif
              {
                this->pub->WaitForConnection();
                this->pub->Publish(*this->msg, true);
                this->pub->SendMessage();
                delete this->msg;
#if TBB_VERSION_MAJOR < 2021
                this->pub.reset();
                return NULL;
#endif
              }

      /// \brief Pointer to the publisher.
      private: transport::PublisherPtr pub;

      /// \brief Message to publish
      private: google::protobuf::Message *msg;
    };
    /// \endcond

    /// \addtogroup gazebo_transport
    /// \{

    /// \class Node Node.hh transport/transport.hh
    /// \brief A node can advertise and subscribe topics, publish on
    ///        advertised topics and listen to subscribed topics.
    class GZ_TRANSPORT_VISIBLE Node :
      public boost::enable_shared_from_this<Node>
    {
      /// \brief Constructor
      public: Node();

      /// \brief Destructor
      public: virtual ~Node();

      /// \brief Init the node
      /// \param[in] _space Set the namespace of this topic. If this is the
      /// first Node initialized, then this namespace will become the global
      /// namespace. If left blank, the topic will initialize to the first
      /// namespace on the Master. If the Master does not have any namespaces
      /// within 1 second of calling this function, the Node will be initialized
      /// with a namespace of "default".
      /// \sa TryInit()
      public: void Init(const std::string &_space ="");

      /// \brief Try to initialize the node to use the global namespace, and
      /// specify the maximum wait time. If a global namespace is not available
      /// by the time a duration of _maxWait has transpired, this will return
      /// false, and the Node will not be initialized.
      /// \param[in] _maxWait The maximum amount of time to wait for the Node to
      /// initialize. The initialization might be delayed while waiting for
      /// namespaces to be found. The default is 1 second.
      /// \return True if a global namespace was found, and this node has been
      /// initialized to it. False if a global namespace was not found (the node
      /// will not be initialized in that case).
      /// \sa Init()
      public: bool TryInit(
        const common::Time &_maxWait = common::Time(1, 0));

      /// \brief Check if this Node has been initialized.
      /// \return True if initialized, otherwise false
      /// \sa Init()
      /// \sa TryInit()
      public: bool IsInitialized() const;

      /// \brief Finalize the node
      public: void Fini();

      /// \brief Get the topic namespace for this node
      /// \return The namespace
      public: std::string GetTopicNamespace() const;

      /// \brief Decode a topic name
      /// \param[in] The encoded name
      /// \return The decoded name
      public: std::string DecodeTopicName(const std::string &_topic);

      /// \brief Encode a topic name
      /// \param[in] The decoded name
      /// \return The encoded name
      public: std::string EncodeTopicName(const std::string &_topic);

      /// \brief Get the unique ID of the node
      /// \return The unique ID of the node
      public: unsigned int GetId() const;

      /// \brief Process all publishers, which has each publisher send it's
      /// most recent message over the wire. This is for internal use only
      public: void ProcessPublishers();

      /// \brief Process incoming messages.
      public: void ProcessIncoming();

      /// \brief Return true if a subscriber on a specific topic is latched.
      /// \param[in] _topic Name of the topic to check.
      /// \return True if a latched subscriber exists.
      public: bool HasLatchedSubscriber(const std::string &_topic) const;


      /// \brief A convenience function for a one-time publication of
      /// a message. This is inefficient, compared to
      /// Node::Advertise followed by Publisher::Publish. This function
      /// should only be used when sending a message very infrequently.
      /// \param[in] _topic The topic to advertise
      /// \param[in] _message Message to be published
      public: template<typename M>
              void Publish(const std::string &_topic,
                  const google::protobuf::Message &_message)
              {
                transport::PublisherPtr pub = this->Advertise<M>(_topic);
#if TBB_VERSION_MAJOR < 2021
                PublishTask *task = new(tbb::task::allocate_root())
                  PublishTask(pub, _message);

                tbb::task::enqueue(*task);
                return;
#else
                this->taskGroup.run<PublishTask>(pub, _message);
#endif
              }

      /// \brief Advertise a topic
      /// \param[in] _topic The topic to advertise
      /// \param[in] _queueLimit The maximum number of outgoing messages to
      /// queue for delivery
      /// \param[in] _hz Update rate for the publisher. Units are
      /// 1.0/seconds.
      /// \return Pointer to new publisher object
      public: template<typename M>
      transport::PublisherPtr Advertise(const std::string &_topic,
                                        unsigned int _queueLimit = 1000,
                                        double _hzRate = 0)
      {
        std::string decodedTopic = this->DecodeTopicName(_topic);
        PublisherPtr publisher =
          transport::TopicManager::Instance()->Advertise<M>(
              decodedTopic, _queueLimit, _hzRate);

        boost::mutex::scoped_lock lock(this->publisherMutex);
        publisher->SetNode(shared_from_this());
        this->publishers.push_back(publisher);

        return publisher;
      }

      /// \brief A convenience function for a one-time publication of
      /// a message. This is inefficient, compared to
      /// Node::Advertise followed by Publisher::Publish. This function
      /// should only be used when sending a message very infrequently.
      /// \param[in] _topic The topic to advertise
      /// \param[in] _message Message to be published
      public: void Publish(const std::string &_topic,
                  const google::protobuf::Message &_message)
              {
                transport::PublisherPtr pub = this->Advertise(_topic,
                    _message.GetTypeName());
                pub->WaitForConnection();

                pub->Publish(_message, true);
              }

      /// \brief Advertise a topic
      /// \param[in] _topic The topic to advertise
      /// \param[in] _queueLimit The maximum number of outgoing messages to
      /// queue for delivery
      /// \param[in] _hz Update rate for the publisher. Units are
      /// 1.0/seconds.
      /// \return Pointer to new publisher object
      public: transport::PublisherPtr Advertise(const std::string &_topic,
                                        const std::string &_msgTypeName,
                                        unsigned int _queueLimit = 1000,
                                        double _hzRate = 0)
      {
        std::string decodedTopic = this->DecodeTopicName(_topic);
        PublisherPtr publisher =
          transport::TopicManager::Instance()->Advertise(
              decodedTopic, _msgTypeName, _queueLimit, _hzRate);

        boost::mutex::scoped_lock lock(this->publisherMutex);
        publisher->SetNode(shared_from_this());
        this->publishers.push_back(publisher);

        return publisher;
      }

      /// \brief Subscribe to a topic using a class method as the callback
      /// \param[in] _topic The topic to subscribe to
      /// \param[in] _fp Class method to be called on receipt of new message
      /// \param[in] _obj Class instance to be used on receipt of new message
      /// \param[in] _latching If true, latch latest incoming message;
      /// otherwise don't latch
      /// \return Pointer to new Subscriber object
      public: template<typename M, typename T>
      SubscriberPtr Subscribe(const std::string &_topic,
          void(T::*_fp)(const boost::shared_ptr<M const> &), T *_obj,
          bool _latching = false)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(_topic);
        ops.template Init<M>(decodedTopic, shared_from_this(), _latching);

        {
          using namespace boost::placeholders;
          boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
          this->callbacks[decodedTopic].push_back(CallbackHelperPtr(
                new CallbackHelperT<M>(boost::bind(_fp, _obj, _1), _latching)));
        }

        SubscriberPtr result =
          transport::TopicManager::Instance()->Subscribe(ops);

        result->SetCallbackId(this->callbacks[decodedTopic].back()->GetId());

        return result;
      }

      /// \brief Subscribe to a topic using a bost::function as the callback,
      /// allowing to pass lambdas.
      /// \param[in] _topic The topic to subscribe to
      /// \param[in] _cb Function to be called on receipt of new message
      /// \param[in] _latching If true, latch latest incoming message;
      /// otherwise don't latch
      /// \return Pointer to new Subscriber object
      public: template<typename M>
      SubscriberPtr Subscribe(const std::string &_topic,
          const boost::function<void (const boost::shared_ptr<M const> &)> &_cb,
                     bool _latching = false)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(_topic);
        ops.template Init<M>(decodedTopic, shared_from_this(), _latching);

        {
          boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
          this->callbacks[decodedTopic].push_back(
              CallbackHelperPtr(new CallbackHelperT<M>(_cb, _latching)));
        }

        SubscriberPtr result =
          transport::TopicManager::Instance()->Subscribe(ops);

        result->SetCallbackId(this->callbacks[decodedTopic].back()->GetId());

        return result;
      }

      /// \brief Subscribe to a topic using a bare function as the callback
      /// \param[in] _topic The topic to subscribe to
      /// \param[in] _fp Function to be called on receipt of new message
      /// \param[in] _latching If true, latch latest incoming message;
      /// otherwise don't latch
      /// \return Pointer to new Subscriber object
      public: template<typename M>
      SubscriberPtr Subscribe(const std::string &_topic,
          void(*_fp)(const boost::shared_ptr<M const> &),
                     bool _latching = false)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(_topic);
        ops.template Init<M>(decodedTopic, shared_from_this(), _latching);

        {
          boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
          this->callbacks[decodedTopic].push_back(
              CallbackHelperPtr(new CallbackHelperT<M>(_fp, _latching)));
        }

        SubscriberPtr result =
          transport::TopicManager::Instance()->Subscribe(ops);

        result->SetCallbackId(this->callbacks[decodedTopic].back()->GetId());

        return result;
      }

      /// \brief Subscribe to a topic using a class method as the callback
      /// \param[in] _topic The topic to subscribe to
      /// \param[in] _fp Class method to be called on receipt of new message
      /// \param[in] _obj Class instance to be used on receipt of new message
      /// \param[in] _latching If true, latch latest incoming message;
      /// otherwise don't latch
      /// \return Pointer to new Subscriber object
      template<typename T>
      SubscriberPtr Subscribe(const std::string &_topic,
          void(T::*_fp)(const std::string &), T *_obj,
          bool _latching = false)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(_topic);
        ops.Init(decodedTopic, shared_from_this(), _latching);

        {
          using namespace boost::placeholders;
          boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
          this->callbacks[decodedTopic].push_back(CallbackHelperPtr(
                new RawCallbackHelper(boost::bind(_fp, _obj, _1))));
        }

        SubscriberPtr result =
          transport::TopicManager::Instance()->Subscribe(ops);

        result->SetCallbackId(this->callbacks[decodedTopic].back()->GetId());

        return result;
      }


      /// \brief Subscribe to a topic using a bare function as the callback
      /// \param[in] _topic The topic to subscribe to
      /// \param[in] _fp Function to be called on receipt of new message
      /// \param[in] _latching If true, latch latest incoming message;
      /// otherwise don't latch
      /// \return Pointer to new Subscriber object
      SubscriberPtr Subscribe(const std::string &_topic,
          void(*_fp)(const std::string &), bool _latching = false)
      {
        SubscribeOptions ops;
        std::string decodedTopic = this->DecodeTopicName(_topic);
        ops.Init(decodedTopic, shared_from_this(), _latching);

        {
          boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
          this->callbacks[decodedTopic].push_back(
              CallbackHelperPtr(new RawCallbackHelper(_fp)));
        }

        SubscriberPtr result =
          transport::TopicManager::Instance()->Subscribe(ops);

        result->SetCallbackId(this->callbacks[decodedTopic].back()->GetId());

        return result;
      }

      /// \brief Handle incoming data.
      /// \param[in] _topic Topic for which the data was received
      /// \param[in] _msg The message that was received
      /// \return true if the message was handled successfully, false otherwise
      public: bool HandleData(const std::string &_topic,
                              const std::string &_msg);

      /// \brief Handle incoming msg.
      /// \param[in] _topic Topic for which the data was received
      /// \param[in] _msg The message that was received
      /// \return true if the message was handled successfully, false otherwise
      public: bool HandleMessage(const std::string &_topic, MessagePtr _msg);

      /// \brief Add a latched message to the node for publication.
      ///
      /// This is called when a subscription is connected to a
      /// publication.
      /// \param[in] _topic Name of the topic to publish data on.
      /// \param[in] _msg The message to publish.
      public: void InsertLatchedMsg(const std::string &_topic,
                                    const std::string &_msg);

      /// \brief Add a latched message to the node for publication.
      ///
      /// This is called when a subscription is connected to a
      /// publication.
      /// \param[in] _topic Name of the topic to publish data on.
      /// \param[in] _msg The message to publish.
      public: void InsertLatchedMsg(const std::string &_topic,
                                    MessagePtr _msg);

      /// \brief Get the message type for a topic
      /// \param[in] _topic The topic
      /// \return The message type
      public: std::string GetMsgType(const std::string &_topic) const;

      /// \internal
      /// \brief Remove a callback. This should only be called by
      /// Subscriber.cc
      /// \param[in] _topic Name of the topic.
      /// \param[in] _id Id of the callback.
      public: void RemoveCallback(const std::string &_topic, unsigned int _id);

      /// \internal
      /// \brief Private implementation of Init() and TryInit()
      /// \param[in] _space Namespace to initialize this Node to. Use an empty
      /// string to have the namespace inferred.
      /// \param[in] _maxWait Maximum amount of time to wait for a namespace if
      /// _space is left blank.
      /// \param[in] _fallbackToDefault If true, after _maxWait passes without a
      /// global namespace appearing, the Node will be initialized to have a
      /// namespace of "default".
      /// \return True if this Node is initialized upon returning from this
      /// function.
      private: bool PrivateInit(const std::string &_space,
                                const common::Time &_maxWait,
                                const bool _fallbackToDefault);

      private: std::string topicNamespace;
      private: std::vector<PublisherPtr> publishers;
      private: std::vector<PublisherPtr>::iterator publishersIter;
      private: static unsigned int idCounter;
      private: unsigned int id;

      private: typedef std::list<CallbackHelperPtr> Callback_L;
      private: typedef std::map<std::string, Callback_L> Callback_M;
      private: Callback_M callbacks;
      private: std::map<std::string, std::list<std::string> > incomingMsgs;

      /// \brief List of newly arrive messages
      private: std::map<std::string, std::list<MessagePtr> > incomingMsgsLocal;

#if TBB_VERSION_MAJOR >= 2021
      /// \brief For managing asynchronous tasks with tbb
      private: TaskGroup taskGroup;
#endif

      private: boost::mutex publisherMutex;
      private: boost::mutex publisherDeleteMutex;
      private: boost::recursive_mutex incomingMutex;

      /// \brief make sure we don't call ProcessingIncoming simultaneously
      /// from separate threads.
      private: boost::recursive_mutex processIncomingMutex;

      private: bool initialized;
    };
    /// \}
  }
}
#endif
