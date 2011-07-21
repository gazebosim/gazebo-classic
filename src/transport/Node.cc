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
}

Node::~Node()
{
  TopicManager::Instance()->RemoveNode( shared_from_this() );
}

void Node::Init(const std::string &space)
{
  TopicManager::Instance()->AddNode( shared_from_this() );
  this->topicNamespace = space;
}

std::string Node::DecodeTopicName(const std::string &topic)
{
  std::string result = topic;
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
  std::vector<PublisherPtr>::iterator iter;
  for (iter = this->publishers.begin(); iter != this->publishers.end(); iter++)
  {
    (*iter)->SendMessage();
  }
}
