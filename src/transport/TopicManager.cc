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


#include "msgs/msgs.h"
#include "transport/Node.hh"
#include "transport/Publication.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
TopicManager::TopicManager()
{
  this->nodeMutex = new boost::recursive_mutex();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TopicManager::~TopicManager()
{
  delete this->nodeMutex;
  this->nodeMutex = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Init the topic Manager
void TopicManager::Init()
{
  this->advertisedTopics.clear();
  this->subscribed_topics.clear();
  this->nodes.clear();
}

void TopicManager::Fini()
{
  std::vector<PublicationPtr>::iterator iter;
  for (iter = this->advertisedTopics.begin(); 
       iter != this->advertisedTopics.end(); iter++)
  {
    this->Unadvertise( (*iter)->GetTopic() );
  }

  SubMap::iterator iter2;
  for (iter2 = this->subscribed_topics.begin(); 
       iter2 != this->subscribed_topics.end(); iter2++)
  {
    std::list<CallbackHelperPtr>::iterator iter3;
    while (iter2->second.size() > 0)
    {
      this->Unsubscribe( iter2->first, iter2->second.front() );
    }
  }

  this->advertisedTopics.clear();
  this->subscribed_topics.clear();
  this->nodes.clear();
}

void TopicManager::AddNode( NodePtr _node )
{
  this->nodeMutex->lock();
  this->nodes.push_back(_node);
  this->nodeMutex->unlock();
}

void TopicManager::RemoveNode( unsigned int _id )
{
  std::vector<NodePtr>::iterator iter;
  this->nodeMutex->lock();
  for (iter = this->nodes.begin(); iter != this->nodes.end(); iter++)
  {
    if ( (*iter)->GetId() == _id)
    {
      this->nodes.erase(iter);
      break;
    }
  }
  this->nodeMutex->unlock();
}

///////////////////////////////////////////////////////////////////////////////
// Process all nodes
void TopicManager::ProcessNodes()
{
  std::vector<NodePtr>::iterator iter;

  this->nodeMutex->lock();
  for (iter = this->nodes.begin(); iter != this->nodes.end(); iter++)
  {
    (*iter)->ProcessPublishers();
  }
  this->nodeMutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Send a message
void TopicManager::Publish( const std::string &_topic, 
                            google::protobuf::Message &_message,
                            const boost::function<void()> &_cb)
{
  if (!_message.IsInitialized())
  {
    gzthrow( "Publishing and uninitialized message on topic[" + _topic + "]. Required field [" + _message.InitializationErrorString() + "] missing." );
  }

  PublicationPtr pub = this->FindPublication(_topic);
  PublicationPtr dbgPub = this->FindPublication(_topic+"/__dbg");

  pub->Publish( _message, _cb ); 

  if (dbgPub->GetCallbackCount() > 0)
  {
    msgs::String dbgMsg;
    dbgMsg.set_data(_message.DebugString());
    dbgPub->Publish( dbgMsg ); 
  }
}

////////////////////////////////////////////////////////////////////////////////
PublicationPtr TopicManager::FindPublication(const std::string &_topic)
{
  std::vector<PublicationPtr>::iterator iter;
  PublicationPtr pub;

  // Find the publication
  for (iter = this->advertisedTopics.begin(); 
      iter != this->advertisedTopics.end(); iter++)
  {
    if ((*iter)->GetTopic() == _topic)
    {
      pub = *iter;
      break;
    }
  }

  return pub;
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe to a topic give some options
SubscriberPtr TopicManager::Subscribe(const SubscribeOptions &ops)
{
  CallbackHelperPtr subscription = ops.GetSubscription();

  // Create a subscription (essentially a callback that gets 
  // fired every time a Publish occurs on the corresponding
  // topic
  //CallbackHelperPtr subscription( new CallbackHelperT<M>( callback ) );
  //this->subscribed_topics[topic].push_back(subscription);
  
  this->subscribed_topics[ops.GetTopic()].push_back(subscription);

  // The object that gets returned to the caller of this
  // function
  SubscriberPtr sub( new Subscriber(ops.GetTopic(), subscription) );

  // Find a current publication
  PublicationPtr pub = this->FindPublication(ops.GetTopic());

  // If the publication exits, just add the subscription to it 
  if (pub)
  {
    pub->AddSubscription( subscription );
  }
  else if (ops.GetTopic() == "/gazebo/default/selection")
    gzdbg << "No publication\n";

  // Use this to find other remote publishers
  ConnectionManager::Instance()->Subscribe(ops.GetTopic(), ops.GetMsgType());
  return sub;
}


////////////////////////////////////////////////////////////////////////////////
// Handle an incoming message
void TopicManager::HandleIncoming()
{
  //implement this
  // Read a header in the message the indicates the topic
}

////////////////////////////////////////////////////////////////////////////////
// Unsubscribe from a topic
void TopicManager::Unsubscribe( const std::string &_topic, 
                                const CallbackHelperPtr &_sub)
{
  PublicationPtr publication = this->FindPublication(_topic);
  if (publication)
  {
    publication->RemoveSubscription(_sub);
    ConnectionManager::Instance()->Unsubscribe(_topic, _sub->GetMsgType() );
  }

  this->subscribed_topics[_topic].remove( _sub );
}

////////////////////////////////////////////////////////////////////////////////
// Connect a local Publisher to a remote Subscriber
void TopicManager::ConnectPubToSub( const std::string &topic,
                                    const SubscriptionTransportPtr &sublink )
{
  PublicationPtr publication = this->FindPublication( topic );
  publication->AddSubscription( sublink );
}

////////////////////////////////////////////////////////////////////////////////
// Disconnect a local publisher from a remote subscriber
void TopicManager::DisconnectPubFromSub( const std::string &topic, const std::string &host, unsigned int port)
{
  PublicationPtr publication = this->FindPublication(topic);
  publication->RemoveSubscription(host, port);
}

////////////////////////////////////////////////////////////////////////////////
// Disconnection all local subscribers from a remote publisher
void TopicManager::DisconnectSubFromPub( const std::string &topic, const std::string &host, unsigned int port)
{
  PublicationPtr publication = this->FindPublication(topic);
  if (publication)
    publication->RemoveTransport(host, port);
}

////////////////////////////////////////////////////////////////////////////////
// Connect all subscribers on a topic to known publishers
void TopicManager::ConnectSubscribers(const std::string &_topic)
{
  SubMap::iterator iter = this->subscribed_topics.find(_topic);

  if (iter != this->subscribed_topics.end())
  {
    PublicationPtr publication = this->FindPublication(_topic);
    if (!publication)
      return;

    // Add all of our subscriptions to the publication
    std::list<CallbackHelperPtr>::iterator cbIter;
    for (cbIter = iter->second.begin(); cbIter != iter->second.end(); cbIter++)
    {
      publication->AddSubscription( *cbIter );
    }
  }
  else
    gzerr << "Shouldn't get here topic[" << _topic << "]\n";//TODO: Properly handle this error
}

////////////////////////////////////////////////////////////////////////////////
/// Connect a local subscriber to a remote publisher
void TopicManager::ConnectSubToPub( const msgs::Publish &_pub)
{
  this->UpdatePublications(_pub.topic(), _pub.msg_type());

  PublicationPtr publication = this->FindPublication(_pub.topic());

  if (publication && !publication->HasTransport(_pub.host(), _pub.port()))
  {
    // Connect to the remote publisher
    ConnectionPtr conn = ConnectionManager::Instance()->ConnectToRemoteHost(
        _pub.host(), _pub.port());

    // Create a transport link that will read from the connection, and 
    // send data to a Publication.
    PublicationTransportPtr publink(new PublicationTransport(_pub.topic(), 
          _pub.msg_type()));
    publink->Init( conn );

    publication->AddTransport( publink );
  }

  this->ConnectSubscribers(_pub.topic());
}


////////////////////////////////////////////////////////////////////////////////
// Add a new publication to the list of advertised publication
PublicationPtr TopicManager::UpdatePublications( const std::string &topic, 
                                                 const std::string &msgType )
{
  bool inserted = false;

  // Find a current publication on this topic
  PublicationPtr pub = this->FindPublication(topic);

  if (pub)
  {
    // TODO: Handle this error properly
    if (msgType != pub->GetMsgType())
      gzerr << "Attempting to advertise on an existing topic with a conflicting message type\n";
  }
  else
  {
    PublicationPtr dbgPub;
    msgs::String tmp;

    inserted = true;
    pub = PublicationPtr( new Publication(topic, msgType) );
    dbgPub = PublicationPtr( new Publication(topic+"/__dbg", tmp.GetTypeName()) );
    this->advertisedTopics.push_back( pub );
    this->advertisedTopics.push_back( dbgPub );
  }

  return pub;
}

////////////////////////////////////////////////////////////////////////////////
/// Stop advertising on a topic
void TopicManager::Unadvertise(const std::string &_topic)
{
  std::string t;

  for (int i=0; i < 2; i ++)
  { 
    if (i==0)
      t = _topic;
    else
      t = _topic + "/__dbg";

    PublicationPtr publication = this->FindPublication( t );
    if (publication && publication->GetLocallyAdvertised())
    {
      publication->SetLocallyAdvertised(false);
      ConnectionManager::Instance()->Unadvertise(t);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Register a new topic namespace 
void TopicManager::RegisterTopicNamespace(const std::string &_name)
{
  ConnectionManager::Instance()->RegisterTopicNamespace(_name);
}

////////////////////////////////////////////////////////////////////////////////
/// Get all the topic namespaces
void TopicManager::GetTopicNamespaces(std::list<std::string> &_namespaces)
{
  ConnectionManager::Instance()->GetTopicNamespaces(_namespaces);
}
