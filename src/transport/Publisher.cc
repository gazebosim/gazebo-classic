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
/* Desc: Handles pushing messages out on a named topic
 * Author: Nate Koenig
 */

#include "common/GazeboError.hh"
#include "transport/TopicManager.hh"
#include "transport/Publisher.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Publisher::Publisher(const std::string &topic, const std::string &msg_type)
  : topic(topic), msgType(msg_type)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Publisher::~Publisher()
{
  if (!this->topic.empty())
    TopicManager::Instance()->Unadvertise(this->topic);
}

////////////////////////////////////////////////////////////////////////////////
// Publish a message
void Publisher::Publish(google::protobuf::Message &message )
{
  if (message.GetTypeName() != this->msgType)
    gzthrow("Invalid message type\n");

  TopicManager::Instance()->SendMessage(this->topic, message);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the topic name
std::string Publisher::GetTopic() const
{
  return this->topic;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the message type
std::string Publisher::GetMsgType() const
{
  return this->msgType;
}

