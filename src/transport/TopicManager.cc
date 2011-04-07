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

#include <boost/algorithm/string.hpp>

#include "common/Messages.hh"
#include "transport/Publication.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
TopicManager::TopicManager()
{
  this->topicNamespace = "";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TopicManager::~TopicManager()
{
}

void TopicManager::Init(unsigned short port)
{
}


////////////////////////////////////////////////////////////////////////////////
/// Send a message
void TopicManager::Publish( const std::string &topic, 
                            google::protobuf::Message &message )
{
  if (!message.IsInitialized())
  {
    gzthrow("Simulator::SendMessage Message is not initialized[" + message.InitializationErrorString() + "]");
  }

  PublicationPtr pub = this->FindPublication(topic);

  pub->Publish( common::Message::Package("data", message) );
}

////////////////////////////////////////////////////////////////////////////////
PublicationPtr TopicManager::FindPublication(const std::string &topic)
{
  std::vector<PublicationPtr>::iterator iter;
  PublicationPtr pub;

  // Find the publication
  for (iter = this->advertisedTopics.begin(); 
      iter != this->advertisedTopics.end(); iter++)
  {
    if ((*iter)->GetTopic() == topic)
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
  std::string topic = this->DecodeTopicName( ops.GetTopic() );
  CallbackHelperPtr subscription = ops.GetSubscription();

  // Create a subscription (essentially a callback that gets 
  // fired every time a Publish occurs on the corresponding
  // topic
  //CallbackHelperPtr subscription( new CallbackHelperT<M>( callback ) );
  //this->subscribed_topics[topic].push_back(subscription);
  
  this->subscribed_topics[topic].push_back(subscription);

  // The object that gets returned to the caller of this
  // function
  SubscriberPtr sub( new Subscriber(topic, subscription) );

  // Find a current publication
  PublicationPtr pub = this->FindPublication(topic);

  // If the publication exits, just add the subscription to it 
  if (pub)
  {
    std::cout << "LOCAL CONNECTION\n";
    pub->AddSubscription( subscription );
  }
  else
  {
    std::cout << "NOT LOCAL CONNECTION\n";
    // Otherwise subscribe to the remote topic
    ConnectionManager::Instance()->Subscribe(topic, ops.GetMsgType());
  }

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
void TopicManager::Unsubscribe( const std::string &topic, CallbackHelperPtr sub)
{
  this->subscribed_topics[topic].remove(sub);
}

////////////////////////////////////////////////////////////////////////////////
// Connect a local Publisher to a remote Subscriber
void TopicManager::ConnectPubToSub( const std::string &topic,
                                    const SubscriptionTransportPtr &sublink )
{
  std::cout << "\n\n CONNECT PUB TO SUB \n\n";
  PublicationPtr publication = this->FindPublication( topic );
  publication->AddSubscription( sublink );
}

////////////////////////////////////////////////////////////////////////////////
// Connect all subscribers on a topic to known publishers
void TopicManager::ConnectSubscibers(const std::string &topic)
{
  SubMap::iterator iter = this->subscribed_topics.find(topic);

  if (iter != this->subscribed_topics.end())
  {
    PublicationPtr publication = this->FindPublication(topic);

    // Add all of our subscriptions to the publication
    std::list<CallbackHelperPtr>::iterator cbIter;
    for (cbIter = iter->second.begin(); cbIter != iter->second.end(); cbIter++)
    {
      publication->AddSubscription( *cbIter );
    }
  }
  else
    std::cerr << "Shouldn't get here\n";//TODO: Properly handle this error
}

////////////////////////////////////////////////////////////////////////////////
/// Connect a local subscriber to a remote publisher
void TopicManager::ConnectSubToPub( const std::string &topic,
                                    const PublicationTransportPtr &publink )
{

  std::cout << "\n\n CONNECT SUB TO PUB \n\n";

  // Add the publication transport mechanism to the publication.
  if (publink)
  {
    PublicationPtr publication = this->FindPublication(topic);
    if (publication)
      publication->AddTransport( publink );
    else
      std::cerr << "Attempting to connect a remote publisher...but we don't have a publication. This shouldn't happen\n";
  }

  this->ConnectSubscibers(topic);
}

////////////////////////////////////////////////////////////////////////////////
// Add a new publication to the list of advertised publication
bool TopicManager::UpdatePublications( const std::string &topic, 
                                       const std::string &msgType )
{
  bool inserted = false;

  // Find a current publication on this topic
  PublicationPtr pub = this->FindPublication(topic);

  if (pub)
  {
    // TODO: Handle this error properly
    if (msgType != pub->GetMsgType())
      std::cerr << "Attempting to advertise on an existing topic with a conflicting message type\n";
  }
  else
  {
    inserted = true;
    pub = PublicationPtr( new Publication(topic, msgType) );
    this->advertisedTopics.push_back( pub );
  }

  return inserted;
}

std::string TopicManager::DecodeTopicName(const std::string &topic)
{
  std::string result = topic;
  boost::replace_first(result, "~", "/gazebo/" + this->topicNamespace);
  boost::replace_first(result, "//", "/");
  return result;
}

std::string TopicManager::EncodeTopicName(const std::string &topic)
{
  std::string result = topic;
  boost::replace_first(result, "/gazebo/" + this->topicNamespace, "~");
  boost::replace_first(result, "//", "/");

  return result;
}

void TopicManager::SetTopicNamespace(const std::string &space)
{
  this->topicNamespace = space;
}
