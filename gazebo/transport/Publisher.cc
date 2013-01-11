/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"
#include "gazebo/transport/TopicManager.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace transport;

//////////////////////////////////////////////////
Publisher::Publisher(const std::string &_topic, const std::string &_msgType,
                     unsigned int _limit, bool /*_latch*/)
  : topic(_topic), msgType(_msgType), queueLimit(_limit)
{
  this->prevMsg = NULL;
}

//////////////////////////////////////////////////
Publisher::Publisher(const std::string &_topic, const std::string &_msgType,
                     unsigned int _limit)
  : topic(_topic), msgType(_msgType), queueLimit(_limit)
{
  this->prevMsg = NULL;
}

//////////////////////////////////////////////////
Publisher::~Publisher()
{
  if (this->messages.size() > 0)
    this->SendMessage();

  if (!this->topic.empty())
    TopicManager::Instance()->Unadvertise(this->topic);
}

//////////////////////////////////////////////////
bool Publisher::HasConnections() const
{
  return ((this->publications[0] &&
           (this->publications[0]->GetCallbackCount() > 0 ||
            this->publications[0]->GetNodeCount() > 0)) ||
          (this->publications[1] &&
           (this->publications[1]->GetCallbackCount() > 0 ||
            this->publications[1]->GetNodeCount() > 0)));
}

//////////////////////////////////////////////////
void Publisher::WaitForConnection() const
{
  while (!this->HasConnections())
    common::Time::MSleep(100);
}

//////////////////////////////////////////////////
void Publisher::PublishImpl(const google::protobuf::Message &_message,
                            bool /*_block*/)
{
  if (_message.GetTypeName() != this->msgType)
    gzthrow("Invalid message type\n");

  if (!_message.IsInitialized())
  {
    gzthrow("Publishing an uninitialized message on topic[" +
        this->topic + "]. Required field [" +
        _message.InitializationErrorString() + "] missing.");
  }

  // if (!this->HasConnections())
  // return;

  // Save the latest message
  google::protobuf::Message *msg = _message.New();
  msg->CopyFrom(_message);

  {
    boost::recursive_mutex::scoped_lock lock(this->mutex);
    if (this->prevMsg == NULL)
      this->prevMsg = _message.New();
    this->prevMsg->CopyFrom(_message);

    this->messages.push_back(msg);

    if (this->messages.size() > this->queueLimit)
    {
      delete this->messages.front();
      this->messages.pop_front();
    }
  }
}

//////////////////////////////////////////////////
void Publisher::SendMessage()
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);

  if (this->messages.size() > 0)
  {
    std::list<google::protobuf::Message *>::iterator iter;
    for (iter = this->messages.begin(); iter != this->messages.end(); ++iter)
    {
      // Send the latest message.
      TopicManager::Instance()->Publish(this->topic, **iter,
          boost::bind(&Publisher::OnPublishComplete, this));
      delete *iter;
    }

    this->messages.clear();
  }
}

//////////////////////////////////////////////////
unsigned int Publisher::GetOutgoingCount() const
{
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  unsigned int c = this->messages.size();
  return c;
}

//////////////////////////////////////////////////
std::string Publisher::GetTopic() const
{
  return this->topic;
}

//////////////////////////////////////////////////
std::string Publisher::GetMsgType() const
{
  return this->msgType;
}

//////////////////////////////////////////////////
void Publisher::OnPublishComplete()
{
}

//////////////////////////////////////////////////
void Publisher::SetPublication(PublicationPtr &_publication, int _i)
{
  this->publications[_i] = _publication;
}

//////////////////////////////////////////////////
bool Publisher::GetLatching() const
{
  return false;
}

//////////////////////////////////////////////////
std::string Publisher::GetPrevMsg() const
{
  std::string result;
  boost::recursive_mutex::scoped_lock lock(this->mutex);
  if (this->prevMsg)
    this->prevMsg->SerializeToString(&result);
  return result;
}
