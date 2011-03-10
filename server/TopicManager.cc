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
#include "TopicManager.hh"

using namespace gazebo;

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

////////////////////////////////////////////////////////////////////////////////
/// Send a message
void TopicManager::SendMessage( const std::string &topic, 
                                google::protobuf::Message &message )
{
  std::string out;

  // Stamp the message with the transport time
  Message::SendStamp(message);

  if (!message.IsInitialized())
    gzthrow("Simulator::SendMessage Message is not initialized");

  if (!message.SerializeToString(&out))
    gzthrow("Simulator::SendMessage Failed to serialized message");

  std::list<SubscriptionPtr>::iterator iter;
  for (iter = this->subscribed_topics[topic].begin(); 
       iter != this->subscribed_topics[topic].end(); iter++)
  {
    (*iter)->HandleMessage(out);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Unsubscribe from a topic
void TopicManager::Unsubscribe( const std::string &topic, SubscriptionPtr sub)
{
  this->subscribed_topics[topic].remove(sub);
}
