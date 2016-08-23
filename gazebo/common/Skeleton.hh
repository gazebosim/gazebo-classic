/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SKELETON_HH_
#define _GAZEBO_SKELETON_HH_

#include <vector>
#include <string>
#include <map>
#include <utility>

#include <ignition/math/Matrix4.hh>

#include "gazebo/math/Matrix4.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"

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

    /// \class Skeleton Skeleton.hh common/common.hh
    /// \brief A skeleton
    class GZ_COMMON_VISIBLE Skeleton
    {
      /// \brief Constructor
      public: Skeleton();

      /// \brief Constructor
      /// \param[in] _root node
      public: Skeleton(SkeletonNode *_root);

      /// \brief Destructor
      public: virtual ~Skeleton();

      /// \brief Change the root node
      /// \param[in] _node the new node
      public: void SetRootNode(SkeletonNode* _node);

      /// \brief Return the root
      /// \return the root
      public: SkeletonNode* GetRootNode();

      /// \brief Find a node
      /// \param[in] _name the name of the node to look for
      /// \return the node, or NULL if not found
      public: SkeletonNode* GetNodeByName(std::string _name);

      /// \brief Find node by index
      /// \param[in] _id the index
      /// \return the node, or NULL if not found
      public: SkeletonNode* GetNodeById(std::string _id);

      /// \brief Find or create node with handle
      /// \param[in] _handle
      /// \return the node. A new node is created if it didn't exist
      public: SkeletonNode* GetNodeByHandle(unsigned int _handle);

      /// \brief Returns the node count
      /// \return the count
      public: unsigned int GetNumNodes();

      /// \brief Returns the number of joints
      /// \return the count
      public: unsigned int GetNumJoints();

      /// \brief Scale all nodes, transforms and animation data
      /// \param[in] the scaling factor
      public: void Scale(double _scale);

      /// \brief Set the bind pose skeletal transform
      /// \param[in] _trans the transform
      /// \deprecated See SetBindShapeTransform that accepts
      /// ignition::math::Matrix4d
      public: void SetBindShapeTransform(math::Matrix4 _trans)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Set the bind pose skeletal transform
      /// \param[in] _trans the transform
      public: void SetBindShapeTransform(
                  const ignition::math::Matrix4d &_trans);

      /// \brief Return bind pose skeletal transform
      /// \return a matrix
      /// \deprecated See BindShapeTransform that return
      /// ignition::math::Matrix4d.
      public: math::Matrix4 GetBindShapeTransform() GAZEBO_DEPRECATED(6.0);

      /// \brief Return bind pose skeletal transform
      /// \return a matrix
      public: ignition::math::Matrix4d BindShapeTransform();

      /// \brief Outputs the transforms to std::err stream
      public: void PrintTransforms();

      /// \brief Get a copy or the node dictionary
      public: NodeMap GetNodes();

      /// \brief Resizes the raw node weight array
      /// \param[in] _vertices the new size
      public: void SetNumVertAttached(unsigned int _vertices);

      /// \brief Add a new weight to a node (bone)
      /// \param[in] _vertex index of the vertex
      /// \param[in] _node name of the bone
      /// \param[in] _weight the new weight (range 0 to 1)
      public: void AddVertNodeWeight(unsigned int _vertex, std::string _node,
                                     double _weight);

      /// \brief Returns the number of bone weights for a vertex
      /// \param[in] _vertex the index of the vertex
      /// \return the count
      public: unsigned int GetNumVertNodeWeights(unsigned int _vertex);

      /// \brief Weight of a bone for a vertex
      /// \param[in] _v the index of the vertex
      /// \param[in] _i the index of the weight for that vertex
      /// \return a pair containing the name of the node and the weight
      public: std::pair<std::string, double> GetVertNodeWeight(unsigned int _v,
                                     unsigned int _i);

      /// \brief Returns the number of animations
      /// \return the count
      public: unsigned int GetNumAnimations();

      /// \brief Find animation
      /// \param[in] _i the animation index
      /// \return the animation, or NULL if _i is out of bounds
      public: SkeletonAnimation* GetAnimation(const unsigned int _i);

      /// \brief Add an animation. The skeleton does not take ownership of the
      /// animation
      /// \param[in] _anim the animation to add
      public: void AddAnimation(SkeletonAnimation *_anim);

      /// \brief Initializes the hande numbers for each node in the map
      /// using breadth first traversal
      protected: void BuildNodeMap();

      /// \brief the root node
      protected: SkeletonNode *root;

      /// \brief The dictionary of nodes, indexed by name
      protected: NodeMap nodes;

      /// \brief the bind pose skeletal transform
      protected: ignition::math::Matrix4d bindShapeTransform;

      /// \brief the node weight table
      protected: RawNodeWeights rawNW;

      /// \brief the array of animations
      protected: std::vector<SkeletonAnimation*> anims;
    };

    /// \class SkeletonNode Skeleton.hh common/common.hh
    /// \brief A skeleton node
    class GZ_COMMON_VISIBLE SkeletonNode
    {
      /// \brief enumeration of node types
      public: enum SkeletonNodeType {NODE, JOINT};

      /// \brief Constructor
      /// \param[in] _parent The parent node
      public: SkeletonNode(SkeletonNode* _parent);

      /// \brief Constructor
      /// \param[in] _parent the parent node
      /// \param[in] _name name of node
      /// \param[in] _id Id of node
      /// \param[in] _type The type of this node
      public: SkeletonNode(SkeletonNode* _parent, std::string _name,
                std::string _id, SkeletonNodeType _type = JOINT);

      /// \brief Destructor
      public: virtual ~SkeletonNode();

      /// \brief Change the name
      /// \param[in] _name the new name
      public: void SetName(std::string _name);

      /// \brief Returns the name
      /// \return the name
      public: std::string GetName();

      /// \brief Change the id string
      /// \param[in] _id the new id string
      public: void SetId(std::string _id);

      /// \brief Returns the index
      /// \return the id string
      public: std::string GetId();

      /// \brief Change the skeleton node type
      /// \param[in] _type the new type
      public: void SetType(SkeletonNodeType _type);

      /// \brief Is a joint query
      /// \return true if the skeleton type is a joint, false otherwise
      public: bool IsJoint();

      /// \brief Set a transformation
      /// \param[in] _trans the transformation
      /// \param[in] _updateChildren when true the UpdateChildrenTransforms
      /// operation is performed
      /// \deprecated See SetTransform function that accepts
      /// ignition::math::Matrix4d.
      public: void SetTransform(math::Matrix4 _trans,
                  bool _updateChildren = true) GAZEBO_DEPRECATED(6.0);

      /// \brief Set a transformation
      /// \param[in] _trans the transformation
      /// \param[in] _updateChildren when true the UpdateChildrenTransforms
      /// operation is performed
      public: void SetTransform(const ignition::math::Matrix4d &_trans,
                                bool _updateChildren = true);

      /// \brief Set the model transformation
      /// \param[in] _trans the transformation
      /// \param[in] _updateChildren when true the UpdateChildrenTransforms
      /// operation is performed
      /// \deprecated See SetModelTransform that accepts
      /// ignition::math::Matrix4d.
      public: void SetModelTransform(math::Matrix4 _trans,
                  bool _updateChildren = true) GAZEBO_DEPRECATED(6.0);

      /// \brief Set the model transformation
      /// \param[in] _trans the transformation
      /// \param[in] _updateChildren when true the UpdateChildrenTransforms
      /// operation is performed
      public: void SetModelTransform(const ignition::math::Matrix4d &_trans,
                                     bool _updateChildren = true);

      /// \brief Apply model transformations in order for each node in the tree
      public: void UpdateChildrenTransforms();

      /// \brief Sets the initial transformation
      /// \param[in] _tras the transfromation matrix
      /// \deprecated See SetInitialTransform that accepts
      /// ignition::math::Matrix4d.
      public: void SetInitialTransform(math::Matrix4 _tras)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Sets the initial transformation
      /// \param[in] _tras the transfromation matrix
      public: void SetInitialTransform(const ignition::math::Matrix4d &_tras);

      /// \brief Reset the transformation to the initial transformation
      /// \param[in] _resetChildren when true, performs the operation for every
      /// node in the tree
      public: void Reset(bool _resetChildren);

      /// \brief Get transform relative to parent
      /// \return Transform relative to parent
      /// \deprecated See Transform function that return
      /// ignition::math::Matrix4d.
      public: math::Matrix4 GetTransform() GAZEBO_DEPRECATED(6.0);

      /// \brief Get transform relative to parent
      /// \return Transform relative to parent
      public: ignition::math::Matrix4d Transform();

      /// \brief Set the parent node
      /// \param[in] _parent the new parent
      public: void SetParent(SkeletonNode* _parent);

      /// \brief Returns the parent node
      /// \return the parent
      public: SkeletonNode* GetParent();

      /// \brief Queries wether a node has no parent parent
      /// \return true if the node has no parent, fasle otherwise
      public: bool IsRootNode();

      /// \brief Add a new child
      /// \param[in] _child a child
      public: void AddChild(SkeletonNode* _child);

      /// \brief Returns the children count
      /// \return the count
      public: unsigned int GetChildCount();

      /// \brief Find a child by index
      /// \param[in] _index the index
      /// \return the child skeleton. NO BOUNDS CHECKING
      public: SkeletonNode* GetChild(unsigned int _index);

      /// \brief Get child by name
      /// \param[in] _name the name of the child skeleton
      /// \return the skeleton, or NULL if not found
      public: SkeletonNode* GetChildByName(std::string _name);

      /// \brief Get child by string id
      /// \param[in] _id the string id
      /// \return the child skeleton or NULL if not found
      public: SkeletonNode* GetChildById(std::string _id);

      /// \brief Assign a handle number
      /// \param[in] _h the handle
      public: void SetHandle(unsigned int _h);

      /// \brief Get the handle index
      /// \return the handle index
      public: unsigned int GetHandle();

      /// \brief Assign the inverse of the bind pose skeletal transform
      /// \param[in] _invBM the transform
      /// \deprecated See SetInverseBindTransform that accepts
      /// ignition::math::Matrix4d.
      public: void SetInverseBindTransform(math::Matrix4 _invBM)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Assign the inverse of the bind pose skeletal transform
      /// \param[in] _invBM the transform
      public: void SetInverseBindTransform(
                  const ignition::math::Matrix4d &_invBM);

      /// \brief Retrieve the inverse of the bind pose skeletal transform
      /// \return the transform
      /// \deprecated See InverseBindTransform function that returns
      /// ignition::math::Matrix4d.
      public: math::Matrix4 GetInverseBindTransform() GAZEBO_DEPRECATED(6.0);

      /// \brief Retrieve the inverse of the bind pose skeletal transform
      /// \return the transform
      public: ignition::math::Matrix4d InverseBindTransform();

      /// \brief Retrieve the model transform
      /// \return the transform
      /// \deprecated See ModelTransform function that returns
      /// ignition::math::Matrix4d.
      public: math::Matrix4 GetModelTransform() GAZEBO_DEPRECATED(6.0);

      /// \brief Retrieve the model transform
      /// \return the transform
      public: ignition::math::Matrix4d ModelTransform() const;

      /// \brief Retrieve the raw transformations
      /// \return an array of transformations
      public: std::vector<NodeTransform> GetRawTransforms();

      /// \brief Return the raw transformations count
      /// \return the count
      public: unsigned int GetNumRawTrans();

      /// \brief Find a raw transformation
      /// \param[in] _i the index of the transformation
      /// \return the node transform. NO BOUNDS CHECKING PERFORMED
      public: NodeTransform GetRawTransform(unsigned int _i);

      /// \brief Add a raw transform
      /// \param[in] _t the transform
      public: void AddRawTransform(NodeTransform _t);

      /// \brief Returns a copy of the array of transformations.
      /// \return the array of transform (These are the same as the raw trans)
      public: std::vector<NodeTransform> GetTransforms();

      /// \brief the name of the skeletal node
      protected: std::string name;

      /// \brief a string identifier
      protected: std::string id;

      /// \brief the type fo node
      protected: SkeletonNodeType type;

      /// \brief the transform
      protected: ignition::math::Matrix4d transform;

      /// \brief the initial transformation
      protected: ignition::math::Matrix4d initialTransform;

      /// \brief the model transformation
      protected: ignition::math::Matrix4d modelTransform;

      /// \brief the inverse of the bind pose skeletal transform
      protected: ignition::math::Matrix4d invBindTransform;

      /// \brief the parent node
      protected: SkeletonNode *parent;

      /// \brief the children nodes
      protected: std::vector<SkeletonNode*> children;

      /// \brief handle index number
      protected: unsigned int handle;

      /// \brief the raw transformation
      protected: std::vector<NodeTransform> rawTransforms;
    };

    /// \clas NodeTransform Skeleton.hh common/common.hh
    /// \brief A transformation node
    class GZ_COMMON_VISIBLE NodeTransform
    {
      /// \brief Enumeration of the transform types
      public: enum TransformType {TRANSLATE, ROTATE, SCALE, MATRIX};

      /// \brief Constructor
      /// \param[in] _type the type of transform
      public: NodeTransform(TransformType _type = MATRIX);

      /// \brief Constructor
      /// \param[in] _mat the matrix
      /// \param[in] _sid identifier
      /// \param[in] _type the type of transform
      /// \deprecated See NodeTransform constructor that accepts
      /// ignition::math::Matrix4d.
      public: NodeTransform(math::Matrix4 _mat, std::string _sid = "_default_",
                  TransformType _type = MATRIX) GAZEBO_DEPRECATED(6.0);

      /// \brief Constructor
      /// \param[in] _mat the matrix
      /// \param[in] _sid identifier
      /// \param[in] _type the type of transform
      public: NodeTransform(const ignition::math::Matrix4d &_mat,
                  const std::string &_sid = "_default_",
                  TransformType _type = MATRIX);

      /// \brief Destructor. It does nothing.
      public: ~NodeTransform();

      /// \brief Assign a transformation
      /// \param[in] _mat the transform
      /// \deprecated See Set function that accepts
      /// ignition::math::Matrix4d.
      public: void Set(math::Matrix4 _mat) GAZEBO_DEPRECATED(6.0);

      /// \brief Assign a transformation
      /// \param[in] _mat the transform
      public: void Set(const ignition::math::Matrix4d &_mat);

      /// \brief Set transform type
      /// \param[in] _type the type
      public: void SetType(TransformType _type);

      /// \brief Set the SID
      /// \param[in] _sid the sid
      public: void SetSID(std::string _sid);

      /// \brief Returns the transformation matrix
      /// \return the matrix
      /// \deprecated See GetTransform function that returns
      /// ignition::math::Matrix4d.
      public: math::Matrix4 Get() GAZEBO_DEPRECATED(6.0);

      /// \brief Returns the transformation matrix
      /// \return The transform matrix
      public: ignition::math::Matrix4d GetTransform() const;

      /// \brief Returns the transformation type
      /// \return the type
      public: TransformType GetType();

      /// \brief Returns thr SID
      /// \return the SID
      public: std::string GetSID();

      /// \brief Set a transformation matrix component value
      /// \param[in] _idx the component index
      /// \param[in] _value the value
      public: void SetComponent(unsigned int _idx, double _value);

      /// \brief Set source data values
      /// param[in] _mat the values
      /// \deprecated See SetSourceValues function that accepts
      /// ignition::math::Matrix4d.
      public: void SetSourceValues(math::Matrix4 _mat) GAZEBO_DEPRECATED(6.0);

      /// \brief Set source data values
      /// param[in] _mat the values
      public: void SetSourceValues(const ignition::math::Matrix4d &_mat);

      /// \brief Set source data values
      /// \param[in] _vec Vector to set source data values from.
      /// \deprecated See SetSourceValues function that accepts
      /// ignition::math::Vector3d.
      public: void SetSourceValues(math::Vector3 _vec) GAZEBO_DEPRECATED(6.0);

      /// \brief Set source data values
      /// \param[in] _vec Vector to set source data values from.
      public: void SetSourceValues(const ignition::math::Vector3d &_vec);

      /// \brief Sets source matrix values from roation
      /// \param[in] _axis of rotation
      /// \param[in] _angle of rotation
      /// \deprecated See SetSourceValues function that accepts
      /// ignition::math::Vector3d.
      public: void SetSourceValues(math::Vector3 _axis, double _angle)
              GAZEBO_DEPRECATED(6.0);

      /// \brief Sets source matrix values from roation
      /// \param[in] _axis of rotation
      /// \param[in] _angle of rotation
      public: void SetSourceValues(const ignition::math::Vector3d &_axis,
                 const double _angle);

      /// \brief Sets the transform matrix from the source according to the type
      public: void RecalculateMatrix();

      /// \brief Prints the transform matrix to std::err stream
      public: void PrintSource();

      /// \brief Matrix cast operator
      /// \return the transform
      public: ignition::math::Matrix4d operator()();

      /// \brief Node transform multiplication operator
      /// \param[in] _t a transform
      /// \return transform matrix multiplied by _t's transform
      public: ignition::math::Matrix4d operator*(NodeTransform _t);

      /// \brief Matrix multiplication operator
      /// \param[in] _m a matrix
      /// \return transform matrix multiplied by _m
      /// \deprecated See operator* that accepts ignition::math::Matrix4d.
      public: math::Matrix4 operator* (math::Matrix4 _m) GAZEBO_DEPRECATED(6.0);

      /// \brief Matrix multiplication operator
      /// \param[in] _m a matrix
      /// \return transform matrix multiplied by _m
      public: ignition::math::Matrix4d operator*(
                  const ignition::math::Matrix4d &_m);

      /// \brief the sid
      protected: std::string sid;

      /// \brief transform type
      protected: TransformType type;

      /// \brief transform
      protected: ignition::math::Matrix4d transform;

      /// \brief source data values (can be a matrix, a position or rotation)
      protected: std::vector<double> source;
    };
    /// \}
  }
}
#endif

