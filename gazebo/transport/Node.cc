/*
 * Copyright 2011 Nate Koenig
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
  this->publisherMutex = new boost::recursive_mutex();
  this->incomingMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
Node::~Node()
{
  this->Fini();
  delete this->publisherMutex;
  this->publisherMutex = NULL;

  delete this->incomingMutex;
  this->incomingMutex = NULL;
}

/////////////////////////////////////////////////
void Node::Fini()
{
  TopicManager::Instance()->RemoveNode(this->id);

  this->publisherMutex->lock();
  this->publishers.clear();
  this->publisherMutex->unlock();

  this->incomingMutex->lock();
  this->callbacks.clear();
  this->incomingMutex->unlock();
}

/////////////////////////////////////////////////
void Node::Init(const std::string &_space)
{
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
  this->publisherMutex->lock();
  for (this->publishersIter = this->publishers.begin();
       this->publishersIter != this->publishersEnd; this->publishersIter++)
  {
    (*this->publishersIter)->SendMessage();
  }
  this->publisherMutex->unlock();
}

/////////////////////////////////////////////////
bool Node::HandleData(const std::string &_topic, const std::string &_msg)
{
  this->incomingMutex->lock();
  this->incomingMsgs[_topic].push_back(_msg);
  this->incomingMutex->unlock();
  return true;
}

/////////////////////////////////////////////////
void Node::ProcessIncoming()
{
  // Hack...
  if (!this->incomingMutex)
    return;

  Callback_M::iterator cbIter;
  Callback_L::iterator liter;
  std::list<std::string>::iterator msgIter;

  this->incomingMutex->lock();
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
  this->incomingMutex->unlock();
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
