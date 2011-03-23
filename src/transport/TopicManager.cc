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
#include "transport/Client.hh"
#include "transport/Server.hh"
#include "transport/TopicManager.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
TopicManager::TopicManager()
  : server(NULL), client(NULL)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TopicManager::~TopicManager()
{
}

void TopicManager::Init(unsigned short port)
{
  try
  {
    this->server = new Server(port);
  }
  catch (std::exception &e)
  {
    gzthrow( "Unable to start server[" << e.what() << "]\n");
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Send a message
void TopicManager::SendMessage( const std::string &topic, 
                                google::protobuf::Message &message )
{
  if (!message.IsInitialized())
  {
    gzthrow("Simulator::SendMessage Message is not initialized[" + message.InitializationErrorString() + "]");
  }

  msgs::Packet packet;

  common::Time tm = common::Time::GetWallTime();
  packet.mutable_stamp()->set_sec(tm.sec);
  packet.mutable_stamp()->set_nsec(tm.nsec);
  packet.set_topic(topic);

  std::string *serialized_data = packet.mutable_serialized_data();
  if (!message.SerializeToString(serialized_data))
    gzthrow("Failed to serialized message");

  std::cout << "TopicManager Sending a message\n";
  this->server->Write(packet);
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
  this->subscribed_topics[topic].remove(sub);
}
