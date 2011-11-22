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

#include <boost/algorithm/string.hpp>
#include "Node.hh"

using namespace gazebo;
using namespace transport;

unsigned int Node::idCounter = 0;

Node::Node()
{
  this->id = idCounter++;
  this->topicNamespace = "";
  this->publisherMutex = new boost::recursive_mutex();
}

Node::~Node()
{
  this->publishers.clear();
  delete this->publisherMutex;
  this->publisherMutex = NULL;
  TopicManager::Instance()->RemoveNode( this->id );
}

void Node::Init(const std::string &_space)
{
  this->topicNamespace = _space;

  if (_space.empty())
  {
    this->topicNamespace = "default";
    std::list<std::string> namespaces;
    TopicManager::Instance()->GetTopicNamespaces(namespaces);

    std::string ns;
    if (namespaces.size() == 0)
    {
      ns = "default";
      gzerr << "No namespace found";
    }
    else
      ns = namespaces.front();

    this->topicNamespace = namespaces.front();
  }
  else
    TopicManager::Instance()->RegisterTopicNamespace(_space);

  TopicManager::Instance()->AddNode(shared_from_this());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the topic namespace for this node
std::string Node::GetTopicNamespace() const
{
  return this->topicNamespace;
}

std::string Node::DecodeTopicName(const std::string &_topic)
{
  std::string result = _topic;
  boost::replace_first(result, "~", "/gazebo/" + this->topicNamespace);
  boost::replace_first(result, "//", "/");
  return result;
}

std::string Node::EncodeTopicName(const std::string &topic)
{
  std::string result = topic;
  boost::replace_first(result, "/gazebo/" + this->topicNamespace, "~");
  boost::replace_first(result, "//", "/");

  return result;
}

/// Get the unique ID of the node
unsigned int Node::GetId() const
{
  return this->id;
}

void Node::ProcessPublishers()
{
  this->publisherMutex->lock();
  for (this->publishersIter = this->publishers.begin(); 
       this->publishersIter != this->publishersEnd; this->publishersIter++)
  {
    (*this->publishersIter)->SendMessage();
  }
  this->ProcessIncomingMsgs();
  this->publisherMutex->unlock();
}

bool Node::HandleData(const std::string &_topic, const std::string &_msg)
{
  this->publisherMutex->lock();
  this->incomingMsgs[_topic].push_back(_msg);
  this->publisherMutex->unlock();
  return true;
}

void Node::ProcessIncomingMsgs()
{
  Callback_M::iterator cbIter;
  Callback_L::iterator liter;
  std::list<std::string>::iterator msgIter;

  // For each topic
  std::map<std::string, std::list<std::string> >::iterator inIter;
  for (inIter = this->incomingMsgs.begin(); inIter != this->incomingMsgs.end();
       inIter++)
  {
    // Find the callbacks for the topic
    cbIter = this->callbacks.find(inIter->first);
    if (cbIter != this->callbacks.end())
    {
      // For each message in the buffer
      for (msgIter = inIter->second.begin(); msgIter != inIter->second.end();
           msgIter++)
      {
        // Send the message to all callbacks
        for (liter = cbIter->second.begin();
             liter != cbIter->second.end(); liter++)
        {
          (*liter)->HandleData(*msgIter);
        }
      }
    }
  }
  this->incomingMsgs.clear();
}
