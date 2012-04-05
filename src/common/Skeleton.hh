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
#ifndef SKELETON_HH
#define SKELETON_HH

#include <vector>
#include <string>
#include <map>
#include <utility>

#include "math/Matrix4.hh"

namespace gazebo
{
  namespace common
  {
    class SkeletonNode;

    typedef std::map<unsigned int, SkeletonNode*> NodeMap;
    typedef std::map<unsigned int, SkeletonNode*>::iterator NodeMapIter;

    typedef std::map<double, math::Matrix4> RawNodeAnimation;
    typedef std::map<double, math::Matrix4>::iterator RawNodeAnimationIter;

    typedef std::map<std::string, RawNodeAnimation> RawAnimation;
    typedef std::map<std::string, RawNodeAnimation>::iterator RawAnimationIter;

    typedef std::map<std::string, RawAnimation> RawAnimationList;
    typedef std::map<std::string, RawAnimation>::iterator RawAnimationListIter;

    typedef std::vector<std::vector<std::pair<std::string, double> > >
                                                                 RawNodeWeights;

    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief A skeleton
    class Skeleton
    {
      /// \brief Constructor
      public: Skeleton();

      public: Skeleton(SkeletonNode *_root);

      /// \brief Destructor
      public: virtual ~Skeleton();

      public: void SetRootNode(SkeletonNode* _node);

      public: SkeletonNode* GetRootNode();

      public: SkeletonNode* GetNodeByName(std::string _name);

      public: SkeletonNode* GetNodeById(std::string _id);

      public: SkeletonNode* GetNodeByHandle(unsigned int _handle);

      public: void SetBindShapeTransform(math::Matrix4 _trans);

      public: math::Matrix4 GetBindShapeTransform();

      public: void PrintTransforms();

      public: NodeMap GetNodes();

      public: void SetNumVertAttached(unsigned int _vertices);

      public: void AddVertNodeWeight(unsigned int _vertex, std::string _node,
                                     double _weight);

      public: unsigned int GetNumVertNodeWeights(unsigned int _vertex);

      public: std::pair<std::string, double> GetVertNodeWeight(unsigned int _v,
                                     unsigned int _i);

      public: unsigned int GetNumAnimations();

      public: RawAnimationListIter GetAnimationListIter();

      public: void AddAnimation(std::string _name, RawAnimation _anim);

      protected: void BuildNodeMap();

      protected: SkeletonNode *root;

      protected: NodeMap nodes;

      protected: math::Matrix4 bindShapeTransform;

      protected: RawNodeWeights rawNW;

      protected: RawAnimationList animations;
    };

    /// \brief A node
    class SkeletonNode
    {
      public: enum SkeletonNodeType {NODE, JOINT};

      /// \brief Constructor
      public: SkeletonNode(SkeletonNode* _parent);

      public: SkeletonNode(SkeletonNode* _parent, std::string _name,
                std::string _id, SkeletonNodeType _type = JOINT);

      /// \brief Destructor
      public: virtual ~SkeletonNode();

      public: void SetName(std::string _name);

      public: std::string GetName();

      public: void SetId(std::string _id);

      public: std::string GetId();

      public: void SetType(SkeletonNodeType _type);

      public: bool IsJoint();

      public: void SetTransform(math::Matrix4 _trans);

      /// \brief Get transform relative to parent
      public: math::Matrix4 GetTransform();

      public: void SetParent(SkeletonNode* _parent);

      public: SkeletonNode* GetParent();

      public: bool IsRootNode();

      public: void AddChild(SkeletonNode* _child);

      public: unsigned int GetChildCount();

      public: SkeletonNode* GetChild(unsigned int _index);

      /// \brief Get child by name
      public: SkeletonNode* GetChildByName(std::string _name);

      /// \brief Get child by id
      public: SkeletonNode* GetChildById(std::string _id);

      public: void SetHandle(unsigned int _h);

      public: unsigned int GetHandle();

      public: void SetInverseBindTransform(math::Matrix4 _invBM);

      public: math::Matrix4 GetInverseBindTransform();

      public: math::Matrix4 GetWorldTransform();

      protected: std::string name;

      protected: std::string id;

      protected: SkeletonNodeType type;

      protected: math::Matrix4 transform;

      protected: math::Matrix4 worldTransform;

      protected: math::Matrix4 invBindTransform;

      protected: SkeletonNode *parent;

      protected: std::vector<SkeletonNode*> children;

      protected: unsigned int handle;
    };
    /// \}
  }
}
#endif

