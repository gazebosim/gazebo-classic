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
    class NodeTransform;
    class SkeletonAnimation;

    typedef std::map<unsigned int, SkeletonNode*> NodeMap;
    typedef std::map<unsigned int, SkeletonNode*>::iterator NodeMapIter;

    typedef std::map<double, std::vector<NodeTransform> > RawNodeAnim;
    typedef std::map<std::string, RawNodeAnim> RawSkeletonAnim;

    typedef std::vector<std::vector<std::pair<std::string, double> > >
                                                              RawNodeWeights;

    /// \addtogroup gazebo_common Common Animation
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

      public: unsigned int GetNumNodes();

      public: unsigned int GetNumJoints();

      public: void Scale(double _scale);

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

      public: SkeletonAnimation* GetAnimation(const unsigned int i);

      public: void AddAnimation(SkeletonAnimation *_anim);

      protected: void BuildNodeMap();

      protected: SkeletonNode *root;

      protected: NodeMap nodes;

      protected: math::Matrix4 bindShapeTransform;

      protected: RawNodeWeights rawNW;

      protected: std::vector<SkeletonAnimation*> anims;
    };

    /// \brief A skeleton node
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

      public: void SetTransform(math::Matrix4 _trans,
                                  bool _updateChildren = true);

      public: void SetModelTransform(math::Matrix4 _trans,
                                  bool _updateChildren = true);

      public: void UpdateChildrenTransforms();

      public: void SetInitialTransform(math::Matrix4 _tras);

      public: void Reset(bool resetChildren);

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

      public: math::Matrix4 GetModelTransform();

      public: std::vector<NodeTransform> GetRawTransforms();

      public: unsigned int GetNumRawTrans();

      public: NodeTransform GetRawTransform(unsigned int _i);

      public: void AddRawTransform(NodeTransform _t);

      public: std::vector<NodeTransform> GetTransforms();

      protected: std::string name;

      protected: std::string id;

      protected: SkeletonNodeType type;

      protected: math::Matrix4 transform;

      protected: math::Matrix4 initialTransform;

      protected: math::Matrix4 modelTransform;

      protected: math::Matrix4 invBindTransform;

      protected: SkeletonNode *parent;

      protected: std::vector<SkeletonNode*> children;

      protected: unsigned int handle;

      protected: std::vector<NodeTransform> rawTransforms;
    };

    class NodeTransform
    {
      public: enum TransformType {TRANSLATE, ROTATE, SCALE, MATRIX};

      public: NodeTransform(TransformType _type = MATRIX);

      public: NodeTransform(math::Matrix4 _mat, std::string _sid = "_default_",
                                                TransformType _type = MATRIX);

      public: ~NodeTransform();

      public: void Set(math::Matrix4 _mat);

      public: void SetType(TransformType _type);

      public: void SetSID(std::string _sid);

      public: math::Matrix4 Get();

      public: TransformType GetType();

      public: std::string GetSID();

      public: void SetComponent(unsigned int _idx, double _value);

      public: void SetSourceValues(math::Matrix4 _mat);

      public: void SetSourceValues(math::Vector3 _vec);

      public: void SetSourceValues(math::Vector3 _axis, double _angle);

      public: void RecalculateMatrix();

      public: void PrintSource();

      public: math::Matrix4 operator() ();

      public: math::Matrix4 operator* (NodeTransform _t);

      public: math::Matrix4 operator* (math::Matrix4 _m);

      protected: std::string sid;

      protected: TransformType type;

      protected: math::Matrix4 transform;

      protected: std::vector<double> source;
    };
    /// \}
  }
}
#endif

