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
#include "transport/Server.hh"
#include "transport/Client.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
TopicManager::TopicManager()
{
  //this->server = new Server(12345);
  this->client = new Client("localhost", "12346");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TopicManager::~TopicManager()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Send a message
void TopicManager::SendMessage( const std::string &topic, 
                                google::protobuf::Message &message )
{
  std::string out;

  // Stamp the message with the transport time
  common::Message::SendStamp(message);

  if (!message.IsInitialized())
  {
    gzthrow("Simulator::SendMessage Message is not initialized[" + message.InitializationErrorString() + "]");
  }

  if (!message.SerializeToString(&out))
    gzthrow("Simulator::SendMessage Failed to serialized message");

  this->server->Write(out);

  /*std::list<SubscriptionPtr>::iterator iter;
  for (iter = this->subscribed_topics[topic].begin(); 
       iter != this->subscribed_topics[topic].end(); iter++)
  {
    (*iter)->HandleMessage(out);
  }
  */
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
void TopicManager::Unsubscribe( const std::string &topic, SubscriptionPtr sub)
{
  this->subscribed_topics[topic].remove(sub);
}
