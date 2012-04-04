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
#include <list>

#include "common/Skeleton.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Skeleton::Skeleton()
{
}

//////////////////////////////////////////////////
Skeleton::Skeleton(SkeletonNode *_root)
{
  this->root = _root;
  this->BuildNodeMap();
}

//////////////////////////////////////////////////
Skeleton::~Skeleton()
{
  delete this->root;
}

//////////////////////////////////////////////////
void Skeleton::SetRootNode(SkeletonNode* _node)
{
  this->root = _node;
  this->BuildNodeMap();
}

//////////////////////////////////////////////////
SkeletonNode* Skeleton::GetRootNode()
{
  return this->root;
}

//////////////////////////////////////////////////
SkeletonNode* Skeleton::GetNodeByName(std::string _name)
{
  for (std::map<unsigned int, SkeletonNode*>::iterator iter =
      this->nodes.begin(); iter != this->nodes.end(); ++iter)
    if (iter->second->GetName() == _name)
      return iter->second;

  return NULL;
}

//////////////////////////////////////////////////
SkeletonNode* Skeleton::GetNodeById(std::string _id)
{
  for (std::map<unsigned int, SkeletonNode*>::iterator iter =
      this->nodes.begin(); iter != this->nodes.end(); ++iter)
    if (iter->second->GetId() == _id)
      return iter->second;

  return NULL;
}

//////////////////////////////////////////////////
SkeletonNode* Skeleton::GetNodeByHandle(unsigned int _handle)
{
  return this->nodes[_handle];
}

//////////////////////////////////////////////////
void Skeleton::BuildNodeMap()
{
  std::list<SkeletonNode*> toVisit;
  toVisit.push_front(this->root);

  unsigned int handle = 0;

  while (!toVisit.empty())
  {
    SkeletonNode *node = toVisit.front();
    toVisit.pop_front();

    for (int i = (node->GetChildCount() - 1); i >= 0; i--)
      toVisit.push_front(node->GetChild(i));

    node->SetHandle(handle);
    this->nodes[handle] = node;
    handle++;
  }
}

//////////////////////////////////////////////////
void Skeleton::SetBindShapeTransform(math::Matrix4 _trans)
{
  this->bindShapeTransform = _trans;
}

//////////////////////////////////////////////////
math::Matrix4 Skeleton::GetBindShapeTransform()
{
  return this->bindShapeTransform;
}

//////////////////////////////////////////////////
void Skeleton::PrintTransforms()
{
  for (std::map<unsigned int, SkeletonNode*>::iterator iter =
      this->nodes.begin(); iter != this->nodes.end(); ++iter)
  {
    SkeletonNode *node = iter->second;
    std::cerr << "---------------\n" << node->GetName() << "\n";
    std::cerr << node->GetWorldTransform() << "\n";

    if (node->IsJoint())
      std::cerr << node->GetInverseBindTransform() << "\n";
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


//////////////////////////////////////////////////
SkeletonNode::SkeletonNode(SkeletonNode* _parent)
{
  this->parent = _parent;

  if (this->parent)
    this->parent->AddChild(this);
}

//////////////////////////////////////////////////
SkeletonNode::SkeletonNode(SkeletonNode* _parent, std::string _name,
                std::string _id, SkeletonNodeType _type)
{
  this->parent = _parent;

  if (this->parent)
    this->parent->AddChild(this);

  this->name = _name;
  this->id = _id;
  this->type = _type;
}

//////////////////////////////////////////////////
SkeletonNode::~SkeletonNode()
{
  this->children.clear();
}

//////////////////////////////////////////////////
void SkeletonNode::SetName(std::string _name)
{
  this->name = _name;
}

//////////////////////////////////////////////////
std::string SkeletonNode::GetName()
{
  return this->name;
}

//////////////////////////////////////////////////
void SkeletonNode::SetId(std::string _id)
{
  this->id = _id;
}

//////////////////////////////////////////////////
std::string SkeletonNode::GetId()
{
  return this->id;
}

//////////////////////////////////////////////////
void SkeletonNode::SetType(SkeletonNodeType _type)
{
  this->type = _type;
}

//////////////////////////////////////////////////
bool SkeletonNode::IsJoint()
{
  if (this->type == JOINT)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
void SkeletonNode::SetTransform(math::Matrix4 _trans)
{
  this->transform = _trans;

  if (this->parent == NULL)
    this->worldTransform = _trans;
  else
    this->worldTransform = this->parent->GetWorldTransform() * _trans;

  /// propagate the change to the children nodes
  std::list<SkeletonNode*> toVisit;
  for (unsigned int i = 0; i < this->children.size(); i++)
    toVisit.push_back(this->children[i]);

  while (!toVisit.empty())
  {
    SkeletonNode *node = toVisit.front();
    toVisit.pop_front();

    for (int i = (node->GetChildCount() - 1); i >= 0; i++)
      toVisit.push_front(node->GetChild(i));

    node->worldTransform = node->GetParent()->worldTransform * node->transform;
  }
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonNode::GetTransform()
{
  return this->transform;
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonNode::GetWorldTransform()
{
  return this->worldTransform;
}

//////////////////////////////////////////////////
void SkeletonNode::SetParent(SkeletonNode* _parent)
{
  this->parent = _parent;
}

//////////////////////////////////////////////////
SkeletonNode* SkeletonNode::GetParent()
{
  return this->parent;
}

//////////////////////////////////////////////////
bool SkeletonNode::IsRootSkeletonNode()
{
  if (!this->parent)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
void SkeletonNode::AddChild(SkeletonNode* _child)
{
  this->children.push_back(_child);
}

//////////////////////////////////////////////////
unsigned int SkeletonNode::GetChildCount()
{
  return this->children.size();
}

//////////////////////////////////////////////////
SkeletonNode* SkeletonNode::GetChild(unsigned int _index)
{
  return this->children[_index];
}

//////////////////////////////////////////////////
SkeletonNode* SkeletonNode::GetChildByName(std::string _name)
{
  for (unsigned int i = 0; i < this->children.size(); i++)
    if (this->children[i]->GetName() == _name)
      return this->children[i];

  return NULL;
}

//////////////////////////////////////////////////
SkeletonNode* SkeletonNode::GetChildById(std::string _id)
{
  for (unsigned int i = 0; i < this->children.size(); i++)
    if (this->children[i]->GetId() == _id)
      return this->children[i];

  return NULL;
}

//////////////////////////////////////////////////
void SkeletonNode::SetHandle(unsigned int _handle)
{
  this->handle = _handle;
}

//////////////////////////////////////////////////
unsigned int SkeletonNode::GetHandle()
{
  return this->handle;
}

//////////////////////////////////////////////////
void SkeletonNode::SetInverseBindTransform(math::Matrix4 _invBM)
{
  this->invBindTransform = _invBM;
}

math::Matrix4 SkeletonNode::GetInverseBindTransform()
{
  return this->invBindTransform;
}
