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

#include <boost/algorithm/string.hpp>
#include "Node.hh"

using namespace gazebo;
using namespace transport;

unsigned int Node::idCounter = 0;

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
  TopicManager::Instance()->RemoveNode(this->id);

  {
    boost::recursive_mutex::scoped_lock lock(this->publisherMutex);
    this->publishers.clear();
  }

  {
    boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
    this->callbacks.clear();
  }
}

/////////////////////////////////////////////////
void Node::Init(const std::string &_space)
{
  // Don't initialize twice.
  if (this->initialized)
  {
    gzerr << "Node is already initialized, skipping Init."
          << "This shouldn't happen...so fix it.\n";
    return;
  }

  this->topicNamespace = _space;

  if (_space.empty())
  {
    this->topicNamespace = "default";
    std::list<std::string> namespaces;
    TopicManager::Instance()->GetTopicNamespaces(namespaces);

    if (namespaces.empty())
      gzerr << "No namespace found\n";

    this->topicNamespace = namespaces.front();
  }
  else
    TopicManager::Instance()->RegisterTopicNamespace(_space);

  TopicManager::Instance()->AddNode(shared_from_this());

  this->initialized = true;
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
  boost::recursive_mutex::scoped_lock lock(this->publisherMutex);
  for (this->publishersIter = this->publishers.begin();
       this->publishersIter != this->publishersEnd; this->publishersIter++)
  {
    (*this->publishersIter)->SendMessage();
  }
}

/////////////////////////////////////////////////
bool Node::HandleData(const std::string &_topic, const std::string &_msg)
{
  boost::recursive_mutex::scoped_lock lock(this->incomingMutex);
  this->incomingMsgs[_topic].push_back(_msg);
  return true;
}

/////////////////////////////////////////////////
void Node::ProcessIncoming()
{
  boost::recursive_mutex::scoped_lock lock(this->incomingMutex);

  Callback_M::iterator cbIter;
  Callback_L::iterator liter;
  std::list<std::string>::iterator msgIter;

  // For each topic
  std::map<std::string, std::list<std::string> >::iterator inIter;
  std::map<std::string, std::list<std::string> >::iterator endIter;
  inIter = this->incomingMsgs.begin();
  endIter = this->incomingMsgs.end();

  for (; inIter != endIter; ++inIter)
  {
    // Find the callbacks for the topic
    cbIter = this->callbacks.find(inIter->first);
    if (cbIter != this->callbacks.end())
    {
      // For each message in the buffer
      for (msgIter = inIter->second.begin(); msgIter != inIter->second.end();
           ++msgIter)
      {
        // Send the message to all callbacks
        for (liter = cbIter->second.begin();
             liter != cbIter->second.end(); ++liter)
        {
          (*liter)->HandleData(*msgIter);
        }
      }
    }
  }
  this->incomingMsgs.clear();
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
        (*liter)->HandleData(_msg);
    }
  }
}

/////////////////////////////////////////////////
std::string Node::GetMsgType(const std::string &_topic) const
{
  Callback_M::const_iterator iter = this->callbacks.find(_topic);
  if (iter != this->callbacks.end())
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
        iter->second.erase(liter);
        (*liter).reset();
        break;
      }
    }
  }
}
