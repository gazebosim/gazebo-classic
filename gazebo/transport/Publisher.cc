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
/* Desc: Handles pushing messages out on a named topic
 * Author: Nate Koenig
 */
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/bind.hpp>

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Exception.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TopicManager.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace transport;

uint32_t Publisher::idCounter = 0;

//////////////////////////////////////////////////
Publisher::Publisher(const std::string &_topic, const std::string &_msgType,
                     unsigned int _limit, double _hzRate)
  : topic(_topic), msgType(_msgType), queueLimit(_limit),
    updatePeriod(0)
{
  if (!ignition::math::equal(_hzRate, 0.0))
    this->updatePeriod = 1.0 / _hzRate;

  this->queueLimitWarned = false;
  this->pubId = 0;
  this->id = ++idCounter;
}

//////////////////////////////////////////////////
Publisher::~Publisher()
{
}

//////////////////////////////////////////////////
bool Publisher::HasConnections() const
{
  return (this->publication &&
      (this->publication->GetCallbackCount() > 0 ||
       this->publication->GetNodeCount() > 0));
}

//////////////////////////////////////////////////
void Publisher::WaitForConnection() const
{
  while (!this->HasConnections())
    common::Time::MSleep(100);
}

//////////////////////////////////////////////////
bool Publisher::WaitForConnection(const common::Time &_timeout) const
{
  common::Time start = common::Time::GetWallTime();
  common::Time curr = common::Time::GetWallTime();

  while (!this->HasConnections() &&
      (_timeout <= 0.0 || curr - start < _timeout))
  {
    common::Time::MSleep(100);
    curr = common::Time::GetWallTime();
  }

  return this->HasConnections();
}

//////////////////////////////////////////////////
void Publisher::PublishImpl(const google::protobuf::Message &_message,
                            bool _block)
{
  if (_message.GetTypeName() != this->msgType)
    gzthrow("Invalid message type\n");

  if (!_message.IsInitialized())
  {
    gzerr << "Publishing an uninitialized message on topic[" <<
      this->topic << "]. Required field [" <<
      _message.InitializationErrorString() << "] missing.\n";
    return;
  }

  // Check if a throttling rate has been set
  if (this->updatePeriod > 0)
  {
    // Get the current time
    this->currentTime = common::Time::GetWallTime();

    // Skip publication if the time difference is less than the update period.
    if (this->prevPublishTime != common::Time(0, 0) &&
        (this->currentTime - this->prevPublishTime).Double() <
        this->updatePeriod)
    {
      return;
    }

    // Set the previous time a message was published
    this->prevPublishTime = this->currentTime;
  }

  // Save the latest message
  MessagePtr msgPtr(_message.New());
  msgPtr->CopyFrom(_message);

  this->publication->SetPrevMsg(this->id, msgPtr);

  {
    boost::mutex::scoped_lock lock(this->mutex);

    this->messages.push_back(msgPtr);

    if (this->messages.size() > this->queueLimit)
    {
      this->messages.pop_front();

      if (!queueLimitWarned)
      {
        gzwarn << "Queue limit reached for topic "
          << this->topic
          << ", deleting message. "
          << "This warning is printed only once." << std::endl;
        queueLimitWarned = true;
      }
    }
  }

  TopicManager::Instance()->AddNodeToProcess(this->node);

  if (_block)
  {
    this->SendMessage();
  }
  else
  {
    // Tell the connection manager that it needs to update
    ConnectionManager::Instance()->TriggerUpdate();
  }
}

//////////////////////////////////////////////////
void Publisher::SendMessage()
{
  std::list<MessagePtr> localBuffer;
  std::list<uint32_t> localIds;

  {
    boost::mutex::scoped_lock lock(this->mutex);
    if (!this->pubIds.empty() || this->messages.empty())
      return;

    for (unsigned int i = 0; i < this->messages.size(); ++i)
    {
      this->pubId = (this->pubId + 1) % 10000;
      this->pubIds[this->pubId] = 0;
      localIds.push_back(this->pubId);
    }

    std::copy(this->messages.begin(), this->messages.end(),
        std::back_inserter(localBuffer));
    this->messages.clear();
  }

  // Only send messages if there is something to send
  if (!localBuffer.empty())
  {
    std::list<uint32_t>::iterator pubIter = localIds.begin();

    // Send all the current messages
    for (std::list<MessagePtr>::iterator iter = localBuffer.begin();
        iter != localBuffer.end(); ++iter, ++pubIter)
    {
      // Send the latest message.
      this->pubIds[*pubIter] = this->publication->Publish(*iter,
          boost::bind(&Publisher::OnPublishComplete, this, _1), *pubIter);

      if (this->pubIds[*pubIter] <= 0)
        this->pubIds.erase(*pubIter);
    }

    // Clear the local buffer.
    localBuffer.clear();
    localIds.clear();
  }
}

//////////////////////////////////////////////////
void Publisher::SetNode(NodePtr _node)
{
  this->node = _node;
}

//////////////////////////////////////////////////
unsigned int Publisher::GetOutgoingCount() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->messages.size();
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
void Publisher::OnPublishComplete(uint32_t _id)
{
  boost::mutex::scoped_lock lock(this->mutex);

  std::map<uint32_t, int>::iterator iter = this->pubIds.find(_id);
  if (iter != this->pubIds.end() && (--iter->second) <= 0)
    this->pubIds.erase(iter);
}

//////////////////////////////////////////////////
void Publisher::SetPublication(PublicationPtr _publication)
{
  this->publication = _publication;
}

//////////////////////////////////////////////////
void Publisher::Fini()
{
  if (!this->messages.empty())
    this->SendMessage();

  if (!this->topic.empty())
    TopicManager::Instance()->Unadvertise(this->topic);

  common::Time slept;

  // Wait for the message to be published
  while (!this->pubIds.empty() && slept < common::Time(1, 0))
  {
    common::Time::MSleep(10);
    slept += common::Time(0, 10000000);
  }

  this->node.reset();
}

//////////////////////////////////////////////////
std::string Publisher::GetPrevMsg() const
{
  std::string result;
  if (this->publication)
  {
    MessagePtr msg = this->publication->GetPrevMsg(this->id);
    if (msg)
      msg->SerializeToString(&result);
  }

  return result;
}

//////////////////////////////////////////////////
MessagePtr Publisher::GetPrevMsgPtr() const
{
  if (this->publication)
    return this->publication->GetPrevMsg(this->id);
  else
    return MessagePtr();
}
