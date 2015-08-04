/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/math/Angle.hh"

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
unsigned int Skeleton::GetNumNodes()
{
  return this->nodes.size();
}

//////////////////////////////////////////////////
unsigned int Skeleton::GetNumJoints()
{
  unsigned int c = 0;
  for (std::map<unsigned int, SkeletonNode*>::iterator iter =
      this->nodes.begin(); iter != this->nodes.end(); ++iter)
    if (iter->second->IsJoint())
      c++;

  return c;
}


//////////////////////////////////////////////////
void Skeleton::Scale(double _scale)
{
  //  scale skeleton structure
  for (NodeMap::iterator iter = this->nodes.begin();
        iter != this->nodes.end(); ++iter)
  {
    SkeletonNode *node = iter->second;
    ignition::math::Matrix4d trans(node->Transform());
    ignition::math::Vector3d pos(trans.Translation());
    trans.Translate(pos * _scale);
    node->SetTransform(trans, false);
  }

  //  update the nodes' model transforms
  this->root->UpdateChildrenTransforms();

  //  scale the animation data
  for (unsigned int i = 0; i < this->anims.size(); ++i)
    this->anims[i]->Scale(_scale);
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
  this->SetBindShapeTransform(_trans.Ign());
}

//////////////////////////////////////////////////
void Skeleton::SetBindShapeTransform(const ignition::math::Matrix4d &_trans)
{
  this->bindShapeTransform = _trans;
}

//////////////////////////////////////////////////
math::Matrix4 Skeleton::GetBindShapeTransform()
{
  return this->BindShapeTransform();
}

//////////////////////////////////////////////////
ignition::math::Matrix4d Skeleton::BindShapeTransform()
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

    for (unsigned int i = 0; i < node->GetNumRawTrans(); ++i)
    {
      NodeTransform nt = node->GetRawTransform(i);
      std::cerr << "\t" << nt.GetSID();
      if (nt.GetType() == NodeTransform::MATRIX)
        std::cerr << " MATRIX\n";
      else
        if (nt.GetType() == NodeTransform::TRANSLATE)
          std::cerr << " TRANSLATE\n";
        else
          if (nt.GetType() == NodeTransform::ROTATE)
            std::cerr << " ROTATE\n";
          else
            std::cerr << " SCALE\n";
      std::cerr << nt() << "\n+++++++++++\n";
    }

    std::cerr << node->ModelTransform() << "\n";

    if (node->IsJoint())
      std::cerr << node->InverseBindTransform() << "\n";
  }
}

//////////////////////////////////////////////////
NodeMap Skeleton::GetNodes()
{
  return this->nodes;
}

//////////////////////////////////////////////////
void Skeleton::SetNumVertAttached(unsigned int _vertices)
{
  this->rawNW.resize(_vertices);
}

//////////////////////////////////////////////////
void Skeleton::AddVertNodeWeight(unsigned int _vertex, std::string _node,
                       double _weight)
{
  this->rawNW[_vertex].push_back(std::make_pair(_node, _weight));
}

//////////////////////////////////////////////////
unsigned int Skeleton::GetNumVertNodeWeights(unsigned int _vertex)
{
  return this->rawNW[_vertex].size();
}

//////////////////////////////////////////////////
std::pair<std::string, double> Skeleton::GetVertNodeWeight(unsigned int _v,
                                     unsigned int _i)
{
  return this->rawNW[_v][_i];
}

//////////////////////////////////////////////////
unsigned int Skeleton::GetNumAnimations()
{
  return this->anims.size();
}

SkeletonAnimation *Skeleton::GetAnimation(const unsigned int _i)
{
  if (_i >= this->anims.size())
    return NULL;

  return this->anims[_i];
}

//////////////////////////////////////////////////
void Skeleton::AddAnimation(SkeletonAnimation *_anim)
{
  this->anims.push_back(_anim);
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
void SkeletonNode::SetTransform(math::Matrix4 _trans, bool _updateChildren)
{
  this->SetTransform(_trans.Ign(), _updateChildren);
}

//////////////////////////////////////////////////
void SkeletonNode::SetTransform(const ignition::math::Matrix4d &_trans,
    bool _updateChildren)
{
  this->transform = _trans;

  if (this->parent == NULL)
    this->modelTransform = _trans;
  else
    this->modelTransform = this->parent->ModelTransform() * _trans;

  /// propagate the change to the children nodes
  if (_updateChildren)
    this->UpdateChildrenTransforms();
}

//////////////////////////////////////////////////
void SkeletonNode::SetInitialTransform(math::Matrix4 _trans)
{
  this->SetInitialTransform(_trans.Ign());
}

//////////////////////////////////////////////////
void SkeletonNode::SetInitialTransform(const ignition::math::Matrix4d &_trans)
{
  this->initialTransform = _trans;
  this->SetTransform(_trans);
}

//////////////////////////////////////////////////
void SkeletonNode::Reset(bool resetChildren)
{
  this->SetTransform(this->initialTransform);

  if (resetChildren)
    for (unsigned int i = 0; i < this->GetChildCount(); ++i)
      this->GetChild(i)->Reset(true);
}

//////////////////////////////////////////////////
void SkeletonNode::UpdateChildrenTransforms()
{
  std::list<SkeletonNode*> toVisit;
  for (unsigned int i = 0; i < this->children.size(); ++i)
    toVisit.push_back(this->children[i]);

  while (!toVisit.empty())
  {
    SkeletonNode *node = toVisit.front();
    toVisit.pop_front();

    for (int i = (node->GetChildCount() - 1); i >= 0; i--)
      toVisit.push_front(node->GetChild(i));

    node->modelTransform = node->GetParent()->modelTransform * node->transform;
  }
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonNode::GetTransform()
{
  return this->Transform();
}

//////////////////////////////////////////////////
ignition::math::Matrix4d SkeletonNode::Transform()
{
  return this->transform;
}

//////////////////////////////////////////////////
void SkeletonNode::SetModelTransform(math::Matrix4 _trans, bool _updateChildren)
{
  this->SetModelTransform(_trans.Ign(), _updateChildren);
}

//////////////////////////////////////////////////
void SkeletonNode::SetModelTransform(
    const ignition::math::Matrix4d &_trans, const bool _updateChildren)
{
  this->modelTransform = _trans;

  if (this->parent == NULL)
  {
    this->transform = _trans;
  }
  else
  {
    ignition::math::Matrix4d invParentTrans =
      this->parent->ModelTransform().Inverse();
    this->transform = invParentTrans * this->modelTransform;
  }

  if (_updateChildren)
    this->UpdateChildrenTransforms();
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonNode::GetModelTransform()
{
  return this->ModelTransform();
}

//////////////////////////////////////////////////
ignition::math::Matrix4d SkeletonNode::ModelTransform() const
{
  return this->modelTransform;
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
bool SkeletonNode::IsRootNode()
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
  for (unsigned int i = 0; i < this->children.size(); ++i)
    if (this->children[i]->GetName() == _name)
      return this->children[i];

  return NULL;
}

//////////////////////////////////////////////////
SkeletonNode* SkeletonNode::GetChildById(std::string _id)
{
  for (unsigned int i = 0; i < this->children.size(); ++i)
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
  this->SetInverseBindTransform(_invBM.Ign());
}

//////////////////////////////////////////////////
void SkeletonNode::SetInverseBindTransform(
    const ignition::math::Matrix4d &_invBM)
{
  this->invBindTransform = _invBM;
}

//////////////////////////////////////////////////
math::Matrix4 SkeletonNode::GetInverseBindTransform()
{
  return this->InverseBindTransform();
}

//////////////////////////////////////////////////
ignition::math::Matrix4d SkeletonNode::InverseBindTransform()
{
  return this->invBindTransform;
}

//////////////////////////////////////////////////
std::vector<NodeTransform> SkeletonNode::GetRawTransforms()
{
  return this->rawTransforms;
}

//////////////////////////////////////////////////
unsigned int SkeletonNode::GetNumRawTrans()
{
  return this->rawTransforms.size();
}

//////////////////////////////////////////////////
NodeTransform SkeletonNode::GetRawTransform(unsigned int _i)
{
  return this->rawTransforms[_i];
}

//////////////////////////////////////////////////
void SkeletonNode::AddRawTransform(NodeTransform _t)
{
  this->rawTransforms.push_back(_t);
}

//////////////////////////////////////////////////
std::vector<NodeTransform> SkeletonNode::GetTransforms()
{
  return this->rawTransforms;
}


//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


//////////////////////////////////////////////////
NodeTransform::NodeTransform(TransformType _type)
{
  this->sid = "_default_";
  this->type = _type;
  this->transform =
    ignition::math::Matrix4d(ignition::math::Matrix4d::Identity);
}

//////////////////////////////////////////////////
NodeTransform::NodeTransform(math::Matrix4 _mat, std::string _sid,
    TransformType _type)
{
  this->sid = _sid;
  this->type = _type;
  this->transform = _mat.Ign();
}

//////////////////////////////////////////////////
NodeTransform::NodeTransform(const ignition::math::Matrix4d &_mat,
    const std::string &_sid, TransformType _type)
{
  this->sid = _sid;
  this->type = _type;
  this->transform = _mat;
}

//////////////////////////////////////////////////
NodeTransform::~NodeTransform()
{
}

//////////////////////////////////////////////////
void NodeTransform::Set(math::Matrix4 _mat)
{
  this->Set(_mat.Ign());
}

//////////////////////////////////////////////////
void NodeTransform::Set(const ignition::math::Matrix4d &_mat)
{
  this->transform = _mat;
}

//////////////////////////////////////////////////
void NodeTransform::SetType(TransformType _type)
{
  this->type = _type;
}

//////////////////////////////////////////////////
void NodeTransform::SetSID(std::string _sid)
{
  this->sid = _sid;
}

//////////////////////////////////////////////////
math::Matrix4 NodeTransform::Get()
{
  return this->GetTransform();
}

//////////////////////////////////////////////////
ignition::math::Matrix4d NodeTransform::GetTransform() const
{
  return this->transform;
}

//////////////////////////////////////////////////
NodeTransform::TransformType NodeTransform::GetType()
{
  return this->type;
}

//////////////////////////////////////////////////
std::string NodeTransform::GetSID()
{
  return this->sid;
}

//////////////////////////////////////////////////
void NodeTransform::SetComponent(unsigned int _idx, double _value)
{
  this->source[_idx] = _value;
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(math::Matrix4 _mat)
{
  this->SetSourceValues(_mat.Ign());
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(const ignition::math::Matrix4d &_mat)
{
  this->source.resize(16);
  unsigned int idx = 0;
  for (unsigned int i = 0; i < 4; ++i)
  {
    for (unsigned int j = 0; j < 4; ++j)
    {
      this->source[idx] = _mat(i, j);
      idx++;
    }
  }
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(math::Vector3 _vec)
{
  this->SetSourceValues(_vec.Ign());
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(const ignition::math::Vector3d &_vec)
{
  this->source.resize(3);
  this->source[0] = _vec.X();
  this->source[1] = _vec.Y();
  this->source[2] = _vec.Z();
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(math::Vector3 _axis, double _angle)
{
  this->SetSourceValues(_axis.Ign(), _angle);
}

//////////////////////////////////////////////////
void NodeTransform::SetSourceValues(
    const ignition::math::Vector3d &_axis, const double _angle)
{
  this->source.resize(4);
  this->source[0] = _axis.X();
  this->source[1] = _axis.Y();
  this->source[2] = _axis.Z();
  this->source[3] = _angle;
}

//////////////////////////////////////////////////
void NodeTransform::RecalculateMatrix()
{
  if (this->type == MATRIX)
  {
    this->transform.Set(this->source[0], this->source[1], this->source[2],
                        this->source[3], this->source[4], this->source[5],
                        this->source[6], this->source[7], this->source[8],
                        this->source[9], this->source[10], this->source[11],
                        this->source[12], this->source[13], this->source[14],
                        this->source[15]);
  }
  else
    if (this->type == TRANSLATE)
    {
      this->transform.Translate(
          ignition::math::Vector3d(this->source[0],
            this->source[1], this->source[2]));
    }
    else
      if (this->type == ROTATE)
      {
        ignition::math::Matrix3d mat;
        mat.Axis(ignition::math::Vector3d(
              this->source[0], this->source[1], this->source[2]),
            IGN_DTOR(this->source[3]));
        this->transform = mat;
      }
      else
      {
        this->transform.Scale(ignition::math::Vector3d(
              this->source[0], this->source[1], this->source[2]));
      }
}

//////////////////////////////////////////////////
ignition::math::Matrix4d NodeTransform::operator()()
{
  return this->transform;
}

//////////////////////////////////////////////////
ignition::math::Matrix4d NodeTransform::operator*(NodeTransform _t)
{
  ignition::math::Matrix4d m;

  m = this->transform * _t();

  return m;
}

//////////////////////////////////////////////////
math::Matrix4 NodeTransform::operator*(math::Matrix4 _m)
{
  math::Matrix4 m;

  m = gazebo::math::Matrix4(this->transform) * _m;

  return m;
}

//////////////////////////////////////////////////
ignition::math::Matrix4d NodeTransform::operator*(
    const ignition::math::Matrix4d &_m)
{
  ignition::math::Matrix4d m;

  m = this->transform * _m;

  return m;
}

//////////////////////////////////////////////////
void NodeTransform::PrintSource()
{
  std::cerr << this->sid;
  for (unsigned int i = 0; i < this->source.size(); ++i)
    std::cerr << " " << this->source[i];
  std::cerr << "\n";
}
