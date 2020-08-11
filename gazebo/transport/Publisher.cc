/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind.hpp>

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Exception.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TopicManager.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace transport;

uint32_t Publisher::idCounter = 0;

namespace gazebo
{
/// \brief Private class for Publisher
class PublisherPrivate
{
  /// \brief Callback when a publish is completed
  /// \param[in] _id ID associated with the publication.
  public: void OnPublishComplete(uint32_t _pubId);

  /// \brief Current publication ids.
  public: std::map<uint32_t, int> pubIds;

  /// \brief Mutex to protect pubIds
  public: std::mutex mutex;
};
}

/// \brief Mutex to protect publisherMap
static std::mutex pubMapMutex;

/// \brief A map of Publisher id and its shared pointer
/// The PublisherPrivate object has to be stored in a global static map
/// and not within the Publisher class. This is because we need to keep the
/// PublisherPrivate object alive for the caller of the OnPublisherComplete,
/// otherwise we run into a situation in which the caller invokes a
/// function of a destroyed object
static std::map<uint32_t, std::shared_ptr<PublisherPrivate>> publisherMap;

//////////////////////////////////////////////////
void PublisherPrivate::OnPublishComplete(uint32_t _id)
{
  try
  {
    // This is the deeply unsatisfying way of dealing with a race
    // condition where the publisher is destroyed before all
    // OnPublishComplete callbacks are fired.
    std::lock_guard<std::mutex> lock(this->mutex);
    std::map<uint32_t, int>::iterator iter = this->pubIds.find(_id);
    if (iter != this->pubIds.end() && (--iter->second) <= 0)
      this->pubIds.erase(iter);
  }
  catch(...)
  {
    return;
  }
}

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

  std::lock_guard<std::mutex> lock(pubMapMutex);
  publisherMap[this->id] = std::make_shared<PublisherPrivate>();
}

//////////////////////////////////////////////////
Publisher::~Publisher()
{
  this->Fini();
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

  std::shared_ptr<PublisherPrivate> pubDataPtr;
  {
    std::lock_guard<std::mutex> lock2(pubMapMutex);
    pubDataPtr = publisherMap[this->id];
  }

  {
    boost::mutex::scoped_lock lock(this->mutex);
    if (this->messages.empty())
      return;

    {
      std::lock_guard<std::mutex> lock2(pubDataPtr->mutex);
      if (!pubDataPtr->pubIds.empty())
        return;
    }

    for (unsigned int i = 0; i < this->messages.size(); ++i)
    {
      this->pubId = (this->pubId + 1) % 10000;

      {
        std::lock_guard<std::mutex> lock2(pubDataPtr->mutex);
        pubDataPtr->pubIds[this->pubId] = 0;
      }

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
      int result = this->publication->Publish(*iter,
          std::bind(&PublisherPrivate::OnPublishComplete,
          pubDataPtr, std::placeholders::_1), *pubIter);

      std::lock_guard<std::mutex> lock2(pubDataPtr->mutex);
      if (result > 0)
        pubDataPtr->pubIds[*pubIter] = result;
      else
        pubDataPtr->pubIds.erase(*pubIter);
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
void Publisher::SetPublication(PublicationPtr _publication)
{
  this->publication = _publication;
}

//////////////////////////////////////////////////
void Publisher::Fini()
{
  if (!this->messages.empty())
    this->SendMessage();
  this->messages.clear();

  if (!this->topic.empty())
    TopicManager::Instance()->Unadvertise(this->topic, this->id);

  {
    std::lock_guard<std::mutex> lock(pubMapMutex);
    publisherMap.erase(this->id);
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

//////////////////////////////////////////////////
uint32_t Publisher::Id() const
{
  return this->id;
}
