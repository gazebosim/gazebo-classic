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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <boost/function.hpp>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publication.hh"
#include "gazebo/transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

/// \brief Class to facilitate parallel processing of nodes.
class NodeProcess_TBB
{
  /// \brief Constructor.
  /// \param[in] _nodes List of nodes to process.
  public: NodeProcess_TBB(std::vector<NodePtr> *_nodes) : nodes(_nodes) {}

  /// \brief Used by TBB during parallel execution.
  /// \param[in] _r Range within this->nodes to process.
  public: void operator() (const tbb::blocked_range<size_t> &_r) const
  {
    for (size_t i = _r.begin(); i != _r.end(); i++)
    {
      (*this->nodes)[i]->ProcessPublishers();
    }
  }

  /// \brief The list of nodes to process.
  private: std::vector<NodePtr> *nodes;
};


//////////////////////////////////////////////////
TopicManager::TopicManager()
{
  this->pauseIncoming = false;
  this->advertisedTopicsEnd = this->advertisedTopics.end();
}

//////////////////////////////////////////////////
TopicManager::~TopicManager()
{
}

//////////////////////////////////////////////////
void TopicManager::Init()
{
  this->advertisedTopics.clear();
  this->advertisedTopicsEnd = this->advertisedTopics.end();
  this->subscribedNodes.clear();
  this->nodes.clear();
}

//////////////////////////////////////////////////
void TopicManager::Fini()
{
  // These two lines make sure that pending messages get sent out
  this->ProcessNodes(true);
  // ConnectionManager::Instance()->RunUpdate();

  PublicationPtr_M::iterator iter;
  for (iter = this->advertisedTopics.begin();
       iter != this->advertisedTopics.end(); ++iter)
  {
    this->Unadvertise(iter->first);
  }

  this->advertisedTopics.clear();
  this->advertisedTopicsEnd = this->advertisedTopics.end();
  this->subscribedNodes.clear();
  this->nodes.clear();
}

//////////////////////////////////////////////////
void TopicManager::AddNode(NodePtr _node)
{
  boost::recursive_mutex::scoped_lock lock(this->nodeMutex);
  this->nodes.push_back(_node);
}

//////////////////////////////////////////////////
void TopicManager::RemoveNode(unsigned int _id)
{
  std::vector<NodePtr>::iterator iter;
  boost::recursive_mutex::scoped_lock lock(this->nodeMutex);

  for (iter = this->nodes.begin(); iter != this->nodes.end(); ++iter)
  {
    if ((*iter)->GetId() == _id)
    {
      // Remove the node from all publications.
      for (PublicationPtr_M::iterator piter = this->advertisedTopics.begin();
           piter != this->advertisedTopics.end(); ++piter)
      {
        piter->second->RemoveSubscription(*iter);
      }

      // Remove the node from all subscriptions.
      for (SubNodeMap::iterator siter = this->subscribedNodes.begin();
           siter != this->subscribedNodes.end(); ++siter)
      {
        std::list<NodePtr>::iterator subIter = siter->second.begin();
        while (subIter != siter->second.end())
        {
          if (*subIter == *iter)
            siter->second.erase(subIter++);
          else
            ++subIter;
        }
      }

      this->nodes.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void TopicManager::AddNodeToProcess(NodePtr _ptr)
{
  if (_ptr)
  {
    boost::mutex::scoped_lock lock(this->processNodesMutex);
    this->nodesToProcess.insert(_ptr);
  }
}

//////////////////////////////////////////////////
void TopicManager::ProcessNodes(bool _onlyOut)
{
  {
    boost::mutex::scoped_lock lock(this->processNodesMutex);
    for (boost::unordered_set<NodePtr>::iterator iter =
        this->nodesToProcess.begin();
        iter != this->nodesToProcess.end(); ++iter)
    {
      (*iter)->ProcessPublishers();
    }
    this->nodesToProcess.clear();
  }

  // Note: In general there are very few nodes. So, parallelization is not
  // needed. Keeping this code for posterity.
  // int s;
  // {
  //   boost::recursive_mutex::scoped_lock lock(this->nodeMutex);
  //   // store as size might change (spawning)
  //   s = this->nodes.size();
  // }
  // Process nodes in parallel
  // try
  // {
  //   boost::mutex::scoped_lock lock(this->processNodesMutex);
  //   tbb::parallel_for(tbb::blocked_range<size_t>(0, this->nodes.size(), 10),
  //       NodeProcess_TBB(&this->nodes));
  // }
  // catch(...)
  // {
  //   /// Failed to process the nodes this time around. But not to
  //   /// worry. This function is called again.
  // }

  if (!this->pauseIncoming && !_onlyOut)
  {
    {
      int s = 0;
      boost::recursive_mutex::scoped_lock lock(this->nodeMutex);
      s = this->nodes.size();

      for (int i = 0; i < s; ++i)
      {
        this->nodes[i]->ProcessIncoming();
        if (this->pauseIncoming)
          break;
      }
    }
  }
}

//////////////////////////////////////////////////
void TopicManager::Publish(const std::string &_topic, MessagePtr _message,
    boost::function<void(uint32_t)> _cb, uint32_t _id)
{
  PublicationPtr pub = this->FindPublication(_topic);

  if (pub)
    pub->Publish(_message, _cb, _id);
  else if (!_cb.empty())
    _cb(_id);
}

//////////////////////////////////////////////////
PublicationPtr TopicManager::FindPublication(const std::string &_topic)
{
  PublicationPtr_M::iterator iter = this->advertisedTopics.find(_topic);
  if (iter != this->advertisedTopicsEnd)
    return iter->second;
  else
    return PublicationPtr();
}

//////////////////////////////////////////////////
SubscriberPtr TopicManager::Subscribe(const SubscribeOptions &_ops)
{
  // Create a subscription (essentially a callback that gets
  // fired every time a Publish occurs on the corresponding
  // topic
  this->subscribedNodes[_ops.GetTopic()].push_back(_ops.GetNode());

  // The object that gets returned to the caller of this
  // function
  SubscriberPtr sub(new Subscriber(_ops.GetTopic(), _ops.GetNode()));

  // Find a current publication
  PublicationPtr pub = this->FindPublication(_ops.GetTopic());

  // If the publication exits, just add the subscription to it
  if (pub)
    pub->AddSubscription(_ops.GetNode());

  // Use this to find other remote publishers
  ConnectionManager::Instance()->Subscribe(_ops.GetTopic(), _ops.GetMsgType(),
                                           _ops.GetLatching());
  return sub;
}

//////////////////////////////////////////////////
void TopicManager::Unsubscribe(const std::string &_topic,
                               const NodePtr &_node)
{
  boost::mutex::scoped_lock lock(this->subscriberMutex);

  PublicationPtr publication = this->FindPublication(_topic);

  if (publication)
    publication->RemoveSubscription(_node);

  ConnectionManager::Instance()->Unsubscribe(_topic,
      _node->GetMsgType(_topic));

  this->subscribedNodes[_topic].remove(_node);
}

//////////////////////////////////////////////////
void TopicManager::ConnectPubToSub(const std::string &_topic,
                                   const SubscriptionTransportPtr _sublink)
{
  PublicationPtr publication = this->FindPublication(_topic);
  publication->AddSubscription(_sublink);
}

//////////////////////////////////////////////////
void TopicManager::DisconnectPubFromSub(const std::string &topic,
    const std::string &host, unsigned int port)
{
  PublicationPtr publication = this->FindPublication(topic);
  publication->RemoveSubscription(host, port);
}

//////////////////////////////////////////////////
void TopicManager::DisconnectSubFromPub(const std::string &topic,
    const std::string &host, unsigned int port)
{
  PublicationPtr publication = this->FindPublication(topic);
  if (publication)
    publication->RemoveTransport(host, port);
}

//////////////////////////////////////////////////
void TopicManager::ConnectSubscribers(const std::string &_topic)
{
  SubNodeMap::iterator nodeIter = this->subscribedNodes.find(_topic);

  if (nodeIter != this->subscribedNodes.end())
  {
    PublicationPtr publication = this->FindPublication(_topic);
    if (!publication)
      return;

    // Add all of our subscriptions to the publication
    std::list<NodePtr>::iterator cbIter;
    for (cbIter = nodeIter->second.begin();
         cbIter != nodeIter->second.end(); ++cbIter)
    {
      publication->AddSubscription(*cbIter);
    }
  }
  else
  {
    // TODO: Properly handle this error
    gzerr << "Shouldn't get here topic[" << _topic << "]\n";
  }
}

//////////////////////////////////////////////////
void TopicManager::ConnectSubToPub(const msgs::Publish &_pub)
{
  boost::mutex::scoped_lock lock(this->subscriberMutex);
  this->UpdatePublications(_pub.topic(), _pub.msg_type());

  PublicationPtr publication = this->FindPublication(_pub.topic());

  if (publication && !publication->HasTransport(_pub.host(), _pub.port()))
  {
    // Connect to the remote publisher
    ConnectionPtr conn = ConnectionManager::Instance()->ConnectToRemoteHost(
        _pub.host(), _pub.port());

    if (conn)
    {
      // Create a transport link that will read from the connection, and
      // send data to a Publication.
      PublicationTransportPtr publink(new PublicationTransport(_pub.topic(),
            _pub.msg_type()));

      bool latched = false;
      SubNodeMap::iterator nodeIter = this->subscribedNodes.find(_pub.topic());

      // Find if any local node has a latched subscriber for the new topic
      // publication transport.
      if (nodeIter != this->subscribedNodes.end())
      {
        std::list<NodePtr>::iterator cbIter;
        for (cbIter = nodeIter->second.begin();
             cbIter != nodeIter->second.end() && !latched; ++cbIter)
        {
          latched = (*cbIter)->HasLatchedSubscriber(_pub.topic());
        }
      }

      publink->Init(conn, latched);

      publication->AddTransport(publink);
    }
  }

  this->ConnectSubscribers(_pub.topic());
}

//////////////////////////////////////////////////
PublicationPtr TopicManager::UpdatePublications(const std::string &_topic,
                                                const std::string &_msgType)
{
  // Find a current publication on this topic
  PublicationPtr pub = this->FindPublication(_topic);

  if (pub)
  {
    if (_msgType != pub->GetMsgType())
      gzthrow("Attempting to advertise on an existing topic with"
              " a conflicting message type\n");
  }
  else
  {
    pub = PublicationPtr(new Publication(_topic, _msgType));
    this->advertisedTopics[_topic] =  pub;
    this->advertisedTopicsEnd = this->advertisedTopics.end();
  }

  return pub;
}

//////////////////////////////////////////////////
void TopicManager::Unadvertise(const std::string &_topic)
{
  std::string t;

  t = _topic;

  PublicationPtr publication = this->FindPublication(t);
  if (publication && publication->GetLocallyAdvertised() &&
      publication->GetTransportCount() == 0)
  {
    publication->SetLocallyAdvertised(false);
    ConnectionManager::Instance()->Unadvertise(t);
  }
}

//////////////////////////////////////////////////
void TopicManager::Unadvertise(const std::string &_topic, const uint32_t _id)
{
  unsigned int pubsSameTopic = 0;
  PublicationPtr publication = this->FindPublication(_topic);
  if (publication)
  {
    publication->RemovePublisher(_id);
    pubsSameTopic = publication->PublisherCount();
  }

  // Unadvertise topic if this was its last publisher
  if (pubsSameTopic == 0)
    this->Unadvertise(_topic);
}

//////////////////////////////////////////////////
void TopicManager::Unadvertise(PublisherPtr _pub)
{
  GZ_ASSERT(_pub, "Unadvertising a NULL Publisher");

  if (_pub)
  {
    this->Unadvertise(_pub->GetTopic(), _pub->Id());
  }
}

//////////////////////////////////////////////////
void TopicManager::RegisterTopicNamespace(const std::string &_name)
{
  ConnectionManager::Instance()->RegisterTopicNamespace(_name);
}

//////////////////////////////////////////////////
void TopicManager::GetTopicNamespaces(std::list<std::string> &_namespaces)
{
  ConnectionManager::Instance()->GetTopicNamespaces(_namespaces);
}

//////////////////////////////////////////////////
void TopicManager::ClearBuffers()
{
  PublicationPtr_M::iterator iter;
  for (iter = this->advertisedTopics.begin();
       iter != this->advertisedTopics.end(); ++iter)
  {
  }
}

//////////////////////////////////////////////////
void TopicManager::PauseIncoming(bool _pause)
{
  this->pauseIncoming = _pause;
}
