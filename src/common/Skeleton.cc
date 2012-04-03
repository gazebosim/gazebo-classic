/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "common/Skeleton.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Skeleton::Skeleton()
{
}

//////////////////////////////////////////////////
Skeleton::~Skeleton()
{
  delete this->root;
}

//////////////////////////////////////////////////
void Skeleton::SetRootNode(Node* _node)
{
  this->root = _node;
}

//////////////////////////////////////////////////
Node* Skeleton::GetRootNode()
{
  return this->root;
}

//////////////////////////////////////////////////
Node* Skeleton::GetNodeByName(std::string _name)
{
  return this->root->GetByName(_name);
}

//////////////////////////////////////////////////
Node* Skeleton::GetNodeById(std::string _name)
{
  return this->root->GetById(_name);
}

//////////////////////////////////////////////////
Node::Node(Node* _parent)
{
  this->parent = _parent;

  if (this->parent)
    this->parent->AddChild(this);
}

//////////////////////////////////////////////////
Node::Node(Node* _parent, std::string _name, std::string _id, NodeType _type)
{
  this->parent = _parent;

  if (this->parent)
    this->parent->AddChild(this);

  this->name = _name;
  this->id = _id;
  this->type = _type;
}

//////////////////////////////////////////////////
Node::~Node()
{
  this->children.clear();
}

//////////////////////////////////////////////////
void Node::SetName(std::string _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
std::string Node::GetName()
{
  return this->name;
}

//////////////////////////////////////////////////
void Node::SetId(std::string _id)
{
  this->id = _id;
}

//////////////////////////////////////////////////
std::string Node::GetId()
{
  return this->id;
}

//////////////////////////////////////////////////
void Node::SetType(NodeType _type)
{
  this->type = _type;
}

//////////////////////////////////////////////////
bool Node::IsJoint()
{
  if (this->type == JOINT)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
void Node::SetTransform(math::Matrix4 _trans)
{
  this->transform = _trans;
}

//////////////////////////////////////////////////
math::Matrix4 Node::GetTransform()
{
  return this->transform;
}

//////////////////////////////////////////////////
math::Matrix4 Node::GetGlobalTransform()
{
  math::Matrix4 trans = transform;
  Node *node = this->parent;

  while (node)
  {
    trans = node->GetTransform() * trans;
    node = node->GetParent();
  }

  return trans;
}

//////////////////////////////////////////////////
void Node::SetParent(Node* _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
Node* Node::GetParent()
{
  return this->parent;
}

//////////////////////////////////////////////////
bool Node::IsRootNode()
{
  if (!this->parent)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
void Node::AddChild(Node* _child)
{
  this->children.push_back(_child);
}

//////////////////////////////////////////////////
unsigned int Node::GetChildCount()
{
  return this->children.size();
}

//////////////////////////////////////////////////
Node* Node::GetChild(unsigned int _index)
{
  return this->children[_index];
}

//////////////////////////////////////////////////
Node* Node::GetChildByName(std::string _name)
{
  for (unsigned int i = 0; i < this->children.size(); i++)
    if (this->children[i]->GetName() == _name)
      return this->children[i];

  return NULL;
}

//////////////////////////////////////////////////
Node* Node::GetChildById(std::string _id)
{
  for (unsigned int i = 0; i < this->children.size(); i++)
    if (this->children[i]->GetId() == _id)
      return this->children[i];

  return NULL;
}

//////////////////////////////////////////////////
Node* Node::GetByName(std::string _name)
{
  if (this->name == _name)
    return this;
  else
  {
    for (unsigned int i = 0; i < this->children.size(); i++)
    {
      Node* node = this->children[i]->GetByName(_name);
      if (node)
        return node;
    }
    return NULL;
  }
}

//////////////////////////////////////////////////
Node* Node::GetById(std::string _id)
{
  if (this->id == _id)
    return this;
  else
  {
    for (unsigned int i = 0; i < this->children.size(); i++)
    {
      Node* node = this->children[i]->GetById(_id);
      if (node)
        return node;
    }
    return NULL;
  }
}
