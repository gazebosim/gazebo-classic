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

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "SubscriptionTransport.hh"
#include "Publication.hh"
#include "Node.hh"

using namespace gazebo;
using namespace transport;

extern void dummy_callback_fn(uint32_t);
unsigned int Publication::idCounter = 0;

//////////////////////////////////////////////////
Publication::Publication(const std::string &_topic, const std::string &_msgType)
  : topic(_topic), msgType(_msgType), locallyAdvertised(false)
{
  this->id = idCounter++;
}

//////////////////////////////////////////////////
Publication::~Publication()
{
  boost::mutex::scoped_lock lock(this->callbackMutex);
  this->publishers.clear();
}

//////////////////////////////////////////////////
void Publication::AddSubscription(const NodePtr &_node)
{
  std::list<NodePtr>::iterator iter, endIter;

  {
    boost::mutex::scoped_lock lock(this->nodeMutex);
    endIter = this->nodes.end();
    iter = std::find(this->nodes.begin(), this->nodes.end(), _node);
  }

  if (iter == endIter)
  {
    boost::mutex::scoped_lock lock(this->nodeMutex);
    this->nodes.push_back(_node);
  }

  boost::mutex::scoped_lock lock(this->callbackMutex);

  // Send latched messages to the subscription.
  for (std::map<uint32_t, MessagePtr>::iterator pubIter =
      this->prevMsgs.begin(); pubIter != this->prevMsgs.end(); ++pubIter)
  {
    if (pubIter->second)
    {
      _node->InsertLatchedMsg(this->topic, pubIter->second);
    }
  }
}

//////////////////////////////////////////////////
void Publication::AddSubscription(const CallbackHelperPtr _callback)
{
  boost::mutex::scoped_lock lock(this->callbackMutex);

  std::list< CallbackHelperPtr >::iterator iter;
  iter = std::find(this->callbacks.begin(), this->callbacks.end(), _callback);

  if (iter == this->callbacks.end())
  {
    this->callbacks.push_back(_callback);

    if (_callback->GetLatching())
    {
      // Send latched messages to the subscription.
      for (std::map<uint32_t, MessagePtr>::iterator pubIter =
          this->prevMsgs.begin(); pubIter != this->prevMsgs.end(); ++pubIter)
      {
        if (pubIter->second)
        {
          _callback->HandleMessage(pubIter->second);
        }
      }
      _callback->SetLatching(false);
    }
  }
}

//////////////////////////////////////////////////
void Publication::SetPrevMsg(uint32_t _pubId, MessagePtr _msg)
{
  boost::mutex::scoped_lock lock(this->callbackMutex);
  this->prevMsgs[_pubId] = _msg;
}

//////////////////////////////////////////////////
void Publication::AddTransport(const PublicationTransportPtr &_publink)
{
  bool add = true;

  // Find an existing publication transport
  std::list<PublicationTransportPtr>::iterator iter;
  for (iter = this->transports.begin(); iter != this->transports.end(); ++iter)
  {
    if ((*iter)->GetTopic() == _publink->GetTopic() &&
        (*iter)->GetMsgType() == _publink->GetMsgType() &&
        (*iter)->GetConnection()->GetRemoteURI() ==
        _publink->GetConnection()->GetRemoteURI())
    {
      add = false;
      break;
    }
  }

  // Don't add a duplicate transport
  if (add)
  {
    _publink->AddCallback(boost::bind(&Publication::LocalPublish, this, _1));
    this->transports.push_back(_publink);
  }
}

//////////////////////////////////////////////////
bool Publication::HasTransport(const std::string &_host, unsigned int _port)
{
  std::list<PublicationTransportPtr>::iterator iter;
  for (iter = this->transports.begin(); iter != this->transports.end(); ++iter)
  {
    if ((*iter)->GetConnection()->GetRemoteAddress() == _host &&
        (*iter)->GetConnection()->GetRemotePort() == _port)
    {
      return true;
    }
  }

  return false;
}

//////////////////////////////////////////////////
void Publication::RemoveTransport(const std::string &host_, unsigned int port_)
{
  std::list<PublicationTransportPtr>::iterator iter;
  iter = this->transports.begin();
  while (iter != this->transports.end())
  {
    if (!(*iter)->GetConnection()->IsOpen() ||
        ((*iter)->GetConnection()->GetRemoteAddress() == host_ &&
         (*iter)->GetConnection()->GetRemotePort() == port_))
    {
      (*iter)->Fini();
      this->transports.erase(iter++);
    }
    else
      ++iter;
  }
}

//////////////////////////////////////////////////
void Publication::RemoveSubscription(const NodePtr &_node)
{
  boost::mutex::scoped_try_lock lock(this->nodeMutex);
  if (!lock)
  {
    boost::mutex::scoped_lock removeLock(this->nodeRemoveMutex);
    this->removeNodes.push_back(_node->GetId());
    return;
  }

  std::list<NodePtr>::iterator iter;

  for (iter = this->nodes.begin(); iter != this->nodes.end(); ++iter)
  {
    if ((*iter)->GetId() == _node->GetId())
    {
      this->nodes.erase(iter);
      break;
    }
  }

  // If no more subscribers, then disconnect from all publishers
  if (this->nodes.empty() && this->callbacks.empty())
  {
    this->transports.clear();
  }
}

//////////////////////////////////////////////////
void Publication::RemoveSubscription(const std::string &_host,
                                     unsigned int _port)
{
  boost::mutex::scoped_try_lock lock(this->callbackMutex);
  if (!lock)
  {
    boost::mutex::scoped_lock removeLock(this->nodeRemoveMutex);
    this->removeCallbacks.push_back(std::make_pair(_host, _port));
    return;
  }

  SubscriptionTransportPtr subptr;
  std::list< CallbackHelperPtr >::iterator iter;

  iter = this->callbacks.begin();
  while (iter != this->callbacks.end())
  {
    subptr = boost::dynamic_pointer_cast<SubscriptionTransport>(*iter);
    std::string host = subptr->GetConnection()->GetRemoteAddress();
    if (!subptr || !subptr->GetConnection()->IsOpen() ||
        ((host.empty() || host == _host) &&
         subptr->GetConnection()->GetRemotePort() == _port))
    {
      subptr.reset();
      iter = this->callbacks.erase(iter);
    }
    else
      ++iter;
  }

  // If no more subscribers, then disconnect from all publishers
  if (this->nodes.empty() && this->callbacks.empty())
  {
    this->transports.clear();
  }
}

//////////////////////////////////////////////////
void Publication::LocalPublish(const std::string &_data)
{
  std::list<NodePtr>::iterator iter, endIter;

  {
    boost::mutex::scoped_lock lock(this->nodeMutex);

    iter = this->nodes.begin();
    endIter = this->nodes.end();
    while (iter != endIter)
    {
      if ((*iter)->HandleData(this->topic, _data))
        ++iter;
      else
        this->nodes.erase(iter++);
    }
  }

  // It's possible that the function pointed to by "HandleData" (above) will in
  // turn call Publication::RemoveSubscription. The following function call
  // will clean up the nodes that have then been marked for removal
  this->RemoveNodes();

  {
    boost::mutex::scoped_lock lock(this->callbackMutex);
    std::list< CallbackHelperPtr >::iterator cbIter;
    cbIter = this->callbacks.begin();
    while (cbIter != this->callbacks.end())
    {
      if ((*cbIter)->IsLocal())
      {
        if ((*cbIter)->HandleData(_data,
              boost::bind(&dummy_callback_fn, _1), 0))
          ++cbIter;
        else
          cbIter = this->callbacks.erase(cbIter);
      }
      else
        ++cbIter;
    }
  }
}

//////////////////////////////////////////////////
int Publication::Publish(MessagePtr _msg, boost::function<void(uint32_t)> _cb,
    uint32_t _id)
{
  int result = 0;
  std::list<NodePtr>::iterator iter, endIter;

  {
    boost::mutex::scoped_lock lock(this->nodeMutex);

    iter = this->nodes.begin();
    endIter = this->nodes.end();
    while (iter != endIter)
    {
      if ((*iter)->HandleMessage(this->topic, _msg))
        ++iter;
      else
        this->nodes.erase(iter++);
    }
  }

  // It's possible that the function pointed to by "HandleData" (above) will in
  // turn call Publication::RemoveSubscription. The following function call
  // will clean up the nodes that have then been marked for removal
  this->RemoveNodes();

  {
    boost::mutex::scoped_lock lock(this->callbackMutex);

    if (!this->callbacks.empty())
    {
      std::string data;
      _msg->SerializeToString(&data);
      std::list<CallbackHelperPtr>::iterator cbIter;
      cbIter = this->callbacks.begin();

      while (cbIter != this->callbacks.end())
      {
        if ((*cbIter)->HandleData(data, _cb, _id))
        {
          ++result;
          ++cbIter;
        }
        else
          this->callbacks.erase(cbIter++);
      }

      if (this->callbacks.empty() && !_cb.empty())
      {
        _cb(_id);
      }
    }
    else if (!_cb.empty())
    {
      _cb(_id);
    }
  }

  return result;
}

//////////////////////////////////////////////////
std::string Publication::GetMsgType() const
{
  return this->msgType;
}

//////////////////////////////////////////////////
unsigned int Publication::GetTransportCount() const
{
  return this->transports.size();
}

//////////////////////////////////////////////////
unsigned int Publication::GetCallbackCount() const
{
  boost::mutex::scoped_lock lock(this->callbackMutex);
  return this->callbacks.size();
}

//////////////////////////////////////////////////
unsigned int Publication::GetNodeCount() const
{
  boost::mutex::scoped_lock lock(this->nodeMutex);
  return this->nodes.size();
}

//////////////////////////////////////////////////
unsigned int Publication::GetRemoteSubscriptionCount()
{
  unsigned int count = 0;

  boost::mutex::scoped_lock lock(this->callbackMutex);
  std::list< CallbackHelperPtr >::iterator iter;
  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); ++iter)
  {
    if (!(*iter)->IsLocal())
      count++;
  }

  return count;
}

//////////////////////////////////////////////////
bool Publication::GetLocallyAdvertised() const
{
  return this->locallyAdvertised;
}

//////////////////////////////////////////////////
void Publication::SetLocallyAdvertised(bool _value)
{
  this->locallyAdvertised = _value;
}

//////////////////////////////////////////////////
void Publication::AddPublisher(PublisherPtr _pub)
{
  boost::mutex::scoped_lock lock(this->callbackMutex);
  this->publishers.push_back(_pub);
}

//////////////////////////////////////////////////
void Publication::RemovePublisher(PublisherPtr _pub)
{
  boost::mutex::scoped_lock lock(this->callbackMutex);

  GZ_ASSERT(_pub, "Received a NULL PublisherPtr");

  // Find the publiser
  std::vector<PublisherPtr>::iterator iter = std::find(
      this->publishers.begin(), this->publishers.end(), _pub);

  if (iter != this->publishers.end())
    this->publishers.erase(iter);
}

//////////////////////////////////////////////////
void Publication::RemoveNodes()
{
  boost::mutex::scoped_lock removeLock(this->nodeRemoveMutex);

  // Remove queued nodes.
  {
    std::list<NodePtr>::iterator nodeIter;

    for (std::list<unsigned int>::iterator iter = this->removeNodes.begin();
        iter != this->removeNodes.end(); ++iter)
    {
      boost::mutex::scoped_lock lock(this->nodeMutex);
      for (nodeIter = this->nodes.begin(); nodeIter != this->nodes.end();
          ++nodeIter)
      {
        if ((*nodeIter)->GetId() == (*iter))
        {
          this->nodes.erase(nodeIter);
          break;
        }
      }
    }
  }


  // Remove queued subscriptions.
  {
    SubscriptionTransportPtr subptr;
    std::list< CallbackHelperPtr >::iterator iter;
    std::list<std::pair<std::string, unsigned int> >::iterator cbIter;

    // Iterate over all the queued subscriptions for removal
    for (cbIter = this->removeCallbacks.begin();
         cbIter != this->removeCallbacks.end(); ++cbIter)
    {
      boost::mutex::scoped_lock lock(this->callbackMutex);

      // Find the callback that matches the host and port information, and
      // remove it.
      iter = this->callbacks.begin();
      while (iter != this->callbacks.end())
      {
        subptr = boost::dynamic_pointer_cast<SubscriptionTransport>(*iter);
        if (!subptr || !subptr->GetConnection()->IsOpen() ||
            (subptr->GetConnection()->GetRemoteAddress() == (*cbIter).first &&
             subptr->GetConnection()->GetRemotePort() == (*cbIter).second))
        {
          this->callbacks.erase(iter++);
        }
        else
          ++iter;
      }
    }
  }

  // Clear the lists.
  this->removeNodes.clear();
  this->removeCallbacks.clear();

  {
    boost::mutex::scoped_lock lock(this->nodeMutex);
    // If no more subscribers, then disconnect from all publishers
    if (this->nodes.empty() && this->callbacks.empty())
    {
      this->transports.clear();
    }
  }
}

//////////////////////////////////////////////////
MessagePtr Publication::GetPrevMsg(uint32_t _pubId)
{
  if (this->prevMsgs.find(_pubId) != this->prevMsgs.end())
    return this->prevMsgs[_pubId];
  else
    return MessagePtr();
}
