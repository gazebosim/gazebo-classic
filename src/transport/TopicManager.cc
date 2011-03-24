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
#include "common/Messages.hh"
#include "transport/PublicationTransport.hh"
#include "transport/Publication.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
TopicManager::TopicManager()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TopicManager::~TopicManager()
{
}

void TopicManager::Init(unsigned short port)
{
  /*try
  {
    this->server = new Server(port);
  }
  catch (std::exception &e)
  {
    gzthrow( "Unable to start server[" << e.what() << "]\n");
  }
  */
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

  std::cout << "TopicManager::Publish \n";
  pub->Publish( common::Message::Package("data", message) );
}

////////////////////////////////////////////////////////////////////////////////
PublicationPtr TopicManager::FindPublication(const std::string &topic)
{
  std::list<PublicationPtr>::iterator iter;
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
// Handle an incoming message
void TopicManager::HandleIncoming()
{
  std::cout << "TopManager::HandleIncoming\n";
  //implement this
  // Read a header in the message the indicates the topic
}

////////////////////////////////////////////////////////////////////////////////
// Unsubscribe from a topic
void TopicManager::Unsubscribe( const std::string &topic, CallbackHelperPtr sub)
{
  std::cout << "Topicmanager::Unsubscribe[" << topic << "]\n";
  this->subscribed_topics[topic].remove(sub);
}

////////////////////////////////////////////////////////////////////////////////
// Connection a local Publisher to a remote Subscriber
void TopicManager::ConnectPubToSub( const msgs::Subscribe &sub,
                                    const ConnectionPtr &connection )
{
  std::cout << "TopicManager::ConnectPubToSub\n";
  PublicationPtr publication = this->FindPublication( sub.topic() );
  PublicationTransportPtr pubLink( new PublicationTransport() );
  pubLink->Init( connection );

  publication->AddSubscription( pubLink );
}

////////////////////////////////////////////////////////////////////////////////
/// Tells the topic manager about a publisher
void TopicManager::ConnectSubToPub( const msgs::Publish &pub,
                                    const ConnectionPtr &connection )
{
  std::cout << "TopicManager::ConnectPubToSub Topic[" << pub.topic() << "]\n";
  std::map<std::string, std::list<CallbackHelperPtr> >::iterator iter;
  iter = this->subscribed_topics.find(pub.topic());

  if (iter != this->subscribed_topics.end())
  {
    PublicationPtr publication = this->FindPublication(pub.topic());

    std::cout << "TopicManager::ConnectSubscriber subscribers found\n";

    // Add all of our subscriptions to the publication
    std::list<CallbackHelperPtr>::iterator cbIter;
    for (cbIter = iter->second.begin(); cbIter != iter->second.end(); cbIter++)
    {
      std::cout << "Adding subscription\n";
      publication->AddSubscription( *cbIter );
    }

    // Start reading messages from the remote publisher
    connection->StartRead(boost::bind(&Publication::Publish, publication, _1));
  }
  else
    std::cerr << "Shouldn't get here\n";//TODO: Properly handle this error
}

////////////////////////////////////////////////////////////////////////////////
// Add a new publication to the list of advertised publication
void TopicManager::UpdatePublications( const std::string &topic, const std::string &msgType )
{
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
    pub = PublicationPtr( new Publication(topic, msgType) );
    this->advertisedTopics.push_back( pub );
  }
}
