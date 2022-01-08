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
#include <boost/algorithm/string.hpp>
#include <boost/bind/bind.hpp>
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

using namespace gazebo;
using namespace transport;

unsigned int Node::idCounter = 0;

extern void dummy_callback_fn(uint32_t);

/////////////////////////////////////////////////
Node::Node()
{
  this->id = idCounter++;
  this->topicNamespace = "";
  this->initialized = false;
}

/////////////////////////////////////////////////
Node::~Node()
{
  this->Fini();
}

/////////////////////////////////////////////////
void Node::Fini()
{
  if (!this->initialized)
    return;

  // Unadvertise all the publishers.
  for (std::vector<PublisherPtr>::iterator iter = this->publishers.begin();
       iter != this->publishers.end(); ++iter)
  {
    (*iter)->Fini();
    TopicManager::Instance()->Unadvertise(*iter);
  }

  this->initialized = false;
  TopicManager::Instance()->RemoveNode(this->id);

  {
    boost::mutex::scoped_lock lock(this->publisherDeleteMutex);
    boost::mutex::scoped_lock lock2(this->publisherMutex);
    this->publishers.clear();
  }

  {
    boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
    this->callbacks.clear();
  }
}

//////////////////////////////////////////////////
bool Node::TryInit(const common::Time &_maxWait)
{
  return this->PrivateInit("", _maxWait, false);
}

/////////////////////////////////////////////////
void Node::Init(const std::string &_space)
{
  this->PrivateInit(_space, common::Time(1, 0), true);
}

/////////////////////////////////////////////////
bool Node::PrivateInit(const std::string &_space,
                       const common::Time &_maxWait,
                       const bool _fallbackToDefault)
{
  if (this->initialized)
  {
    gzerr << "Node is already initialized, skipping initialization. "
          << "This shouldn't happen... so fix it.\n";

    // If the Node is already initialized, return true in order to match the
    // function description.
    return true;
  }

  // Clearing the topicNamespace field shouldn't affect anything, but let's make
  // sure that the assumption made later in this function is valid.
  this->topicNamespace.clear();

  if (_space.empty())
  {
    if (transport::waitForNamespaces(_maxWait))
    {
      // If waitForNamespaces succeeded, then we are guaranteed to have at least
      // one namespace in the list.
      std::list<std::string> namespaces;
      TopicManager::Instance()->GetTopicNamespaces(namespaces);

      GZ_ASSERT(!namespaces.empty(),
                "It should not be possible for namespaces to be empty here");
      this->topicNamespace = namespaces.front();
    }
    else
    {
      gzerr << "No namespaces found\n";

      // If we're coming here from TryInit(), then quit right away.
      if (!_fallbackToDefault)
        return false;
    }
  }

  // If the topic namespace field is empty, then either a global namespace was
  // not found, or we want a specific namespace for this Node.
  if (this->topicNamespace.empty())
  {
    this->topicNamespace = _space.empty() ? "default" : _space;
    TopicManager::Instance()->RegisterTopicNamespace(this->topicNamespace);
  }

  TopicManager::Instance()->AddNode(shared_from_this());
  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool Node::IsInitialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
std::string Node::GetTopicNamespace() const
{
  return this->topicNamespace;
}

/////////////////////////////////////////////////
std::string Node::DecodeTopicName(const std::string &_topic)
{
  std::string result = _topic;
  boost::replace_first(result, "~", "/gazebo/" + this->topicNamespace);
  boost::replace_first(result, "//", "/");
  return result;
}

/////////////////////////////////////////////////
std::string Node::EncodeTopicName(const std::string &topic)
{
  std::string result = topic;
  boost::replace_first(result, "/gazebo/" + this->topicNamespace, "~");
  boost::replace_first(result, "//", "/");

  return result;
}

/////////////////////////////////////////////////
unsigned int Node::GetId() const
{
  return this->id;
}

/////////////////////////////////////////////////
void Node::ProcessPublishers()
{
  if (!this->initialized)
    return;

  int start, end;
  boost::mutex::scoped_lock lock(this->publisherDeleteMutex);
  boost::mutex::scoped_lock lock2(this->publisherMutex);

  start = 0;
  end = this->publishers.size();

  for (int i = start; i < end; ++i)
    this->publishers[i]->SendMessage();
}

/////////////////////////////////////////////////
bool Node::HandleData(const std::string &_topic, const std::string &_msg)
{
  boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
  this->incomingMsgs[_topic].push_back(_msg);
  ConnectionManager::Instance()->TriggerUpdate();
  return true;
}

/////////////////////////////////////////////////
bool Node::HandleMessage(const std::string &_topic, MessagePtr _msg)
{
  boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
  this->incomingMsgsLocal[_topic].push_back(_msg);
  ConnectionManager::Instance()->TriggerUpdate();
  return true;
}

/////////////////////////////////////////////////
void Node::ProcessIncoming()
{
  boost::recursive_mutex::scoped_lock lock(this->processIncomingMutex);

  if (!this->initialized ||
      (this->incomingMsgs.empty() && this->incomingMsgsLocal.empty()))
    return;

  Callback_M::iterator cbIter;
  Callback_L::iterator liter;

  // For each topic
  {
    std::list<std::string>::iterator msgIter;
    std::map<std::string, std::list<std::string> >::iterator inIter;
    std::map<std::string, std::list<std::string> >::iterator endIter;

    boost::recursive_mutex::scoped_lock lock2(this->incomingMutex);
    inIter = this->incomingMsgs.begin();
    endIter = this->incomingMsgs.end();

    for (; inIter != endIter; ++inIter)
    {
      // Find the callbacks for the topic
      cbIter = this->callbacks.find(inIter->first);
      if (cbIter != this->callbacks.end())
      {
        std::list<std::string>::iterator msgInIter;
        std::list<std::string>::iterator msgEndIter;

        msgInIter = inIter->second.begin();
        msgEndIter = inIter->second.end();

        // For each message in the buffer
        for (msgIter = msgInIter; msgIter != msgEndIter; ++msgIter)
        {
          // Send the message to all callbacks
          for (liter = cbIter->second.begin();
              liter != cbIter->second.end(); ++liter)
          {
            using namespace boost::placeholders;
            (*liter)->HandleData(*msgIter,
                boost::bind(&dummy_callback_fn, _1), 0);
          }
        }
      }
    }

    this->incomingMsgs.clear();
  }

  {
    std::list<MessagePtr>::iterator msgIter;
    std::map<std::string, std::list<MessagePtr> >::iterator inIter;
    std::map<std::string, std::list<MessagePtr> >::iterator endIter;

    boost::recursive_mutex::scoped_lock lock2(this->incomingMutex);
    inIter = this->incomingMsgsLocal.begin();
    endIter = this->incomingMsgsLocal.end();

    for (; inIter != endIter; ++inIter)
    {
      // Find the callbacks for the topic
      cbIter = this->callbacks.find(inIter->first);
      if (cbIter != this->callbacks.end())
      {
        std::list<MessagePtr>::iterator msgInIter;
        std::list<MessagePtr>::iterator msgEndIter;

        msgInIter = inIter->second.begin();
        msgEndIter = inIter->second.end();

        // For each message in the buffer
        for (msgIter = msgInIter; msgIter != msgEndIter; ++msgIter)
        {
          // Send the message to all callbacks
          for (liter = cbIter->second.begin();
              liter != cbIter->second.end(); ++liter)
          {
            (*liter)->HandleMessage(*msgIter);
          }
        }
      }
    }

    this->incomingMsgsLocal.clear();
  }
}

//////////////////////////////////////////////////
void Node::InsertLatchedMsg(const std::string &_topic, const std::string &_msg)
{
  // Find the callbacks for the topic
  Callback_M::iterator cbIter = this->callbacks.find(_topic);

  if (cbIter != this->callbacks.end())
  {
    // Send the message to all callbacks
    for (Callback_L::iterator liter = cbIter->second.begin();
         liter != cbIter->second.end(); ++liter)
    {
      if ((*liter)->GetLatching())
      {
        using namespace boost::placeholders;
        (*liter)->HandleData(_msg, boost::bind(&dummy_callback_fn, _1), 0);
        (*liter)->SetLatching(false);
      }
    }
  }
}

//////////////////////////////////////////////////
void Node::InsertLatchedMsg(const std::string &_topic, MessagePtr _msg)
{
  // Find the callbacks for the topic
  Callback_M::iterator cbIter = this->callbacks.find(_topic);

  if (cbIter != this->callbacks.end())
  {
    // Send the message to all callbacks
    for (Callback_L::iterator liter = cbIter->second.begin();
         liter != cbIter->second.end(); ++liter)
    {
      if ((*liter)->GetLatching())
      {
        (*liter)->HandleMessage(_msg);
        (*liter)->SetLatching(false);
      }
    }
  }
}

/////////////////////////////////////////////////
std::string Node::GetMsgType(const std::string &_topic) const
{
  Callback_M::const_iterator iter = this->callbacks.find(_topic);
  if (iter != this->callbacks.end() && !iter->second.empty())
    return iter->second.front()->GetMsgType();

  return std::string();
}

/////////////////////////////////////////////////
bool Node::HasLatchedSubscriber(const std::string &_topic) const
{
  Callback_M::const_iterator iter = this->callbacks.find(_topic);
  if (iter != this->callbacks.end())
    return iter->second.front()->GetLatching();

  return false;
}

/////////////////////////////////////////////////
void Node::RemoveCallback(const std::string &_topic, unsigned int _id)
{
  if (!this->initialized)
    return;

  boost::recursive_mutex::scoped_lock lock(this->incomingMutex);

  // Find the topic list in the map.
  Callback_M::iterator iter = this->callbacks.find(_topic);

  if (iter != this->callbacks.end())
  {
    Callback_L::iterator liter;

    // Find the callback with the correct ID and remove it.
    for (liter = iter->second.begin(); liter != iter->second.end(); ++liter)
    {
      if ((*liter)->GetId() == _id)
      {
        (*liter).reset();
        iter->second.erase(liter);
        break;
      }
    }
  }
}
