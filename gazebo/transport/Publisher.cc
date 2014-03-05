/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TopicManager.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace transport;

//////////////////////////////////////////////////
Publisher::Publisher(const std::string &_topic, const std::string &_msgType,
                     unsigned int _limit, double _hzRate)
  : topic(_topic), msgType(_msgType), queueLimit(_limit),
    updatePeriod(0)
{
  if (!math::equal(_hzRate, 0.0))
    this->updatePeriod = 1.0 / _hzRate;

  this->queueLimitWarned = false;
  this->waiting = false;
  this->pubId = 0;
}

//////////////////////////////////////////////////
Publisher::~Publisher()
{
  if (!this->messages.empty())
    this->SendMessage();

  if (!this->topic.empty())
    TopicManager::Instance()->Unadvertise(this->topic);
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

  {
    boost::mutex::scoped_lock lock(this->mutex);
    if (this->prevMsg == NULL)
      this->prevMsg = msgPtr;

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
      this->pubIds.push_back(this->pubId);
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
      this->publication->Publish(*iter,
          boost::bind(&Publisher::OnPublishComplete, this, _1), *pubIter);
    }

    // Clear the local buffer.
    localBuffer.clear();
    localIds.clear();
  }


  /*MessagePtr msg;
  {
    boost::mutex::scoped_lock lock(this->mutex);
    if (!this->messages.empty() && !this->waiting)
    {
      msg = this->messages.front();
      this->waiting = true;
      this->pubId = (this->pubId + 1) % 10000;
      this->pubIds.insert(this->pubId);
    }
  }

  // Send the latest message.
  if (msg && this->publication)
  {
    this->publication->Publish(msg,
        boost::bind(&Publisher::OnPublishComplete, this, _1), this->pubId);
  }
  */
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

  std::list<uint32_t>::iterator iter =
    std::find(this->pubIds.begin(), this->pubIds.end(), _id);
  if (iter != this->pubIds.end())
  {
    this->pubIds.erase(iter);
    this->waiting = false;
  }
}

//////////////////////////////////////////////////
void Publisher::SetPublication(PublicationPtr _publication)
{
  this->publication = _publication;
}

//////////////////////////////////////////////////
std::string Publisher::GetPrevMsg() const
{
  std::string result;
  boost::mutex::scoped_lock lock(this->mutex);
  if (this->prevMsg)
    this->prevMsg->SerializeToString(&result);
  return result;
}

//////////////////////////////////////////////////
MessagePtr Publisher::GetPrevMsgPtr() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (this->prevMsg)
    return this->prevMsg;
  return MessagePtr();
}
