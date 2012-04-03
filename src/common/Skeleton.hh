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

#include "math/Matrix4.hh"

namespace gazebo
{
  namespace common
  {
    class Node;

    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief A skeleton
    class Skeleton
    {
      /// \brief Constructor
      public: Skeleton();

      /// \brief Destructor
      public: virtual ~Skeleton();

      public: void SetRootNode(Node* _node);

      public: Node* GetRootNode();

      public: Node* GetNodeByName(std::string _name);

      public: Node* GetNodeById(std::string _id);

      protected: Node *root;
    };

    /// \brief A node
    class Node
    {
      public: enum NodeType {NODE, JOINT};

      /// \brief Constructor
      public: Node(Node* _parent);

      public: Node(Node* _parent, std::string _name, std::string _id,
                  NodeType _type = NODE);

      /// \brief Destructor
      public: virtual ~Node();

      public: void SetName(std::string _name);

      public: std::string GetName();

      public: void SetId(std::string _id);

      public: std::string GetId();

      public: void SetType(NodeType _type);

      public: bool IsJoint();

      public: void SetTransform(math::Matrix4 _trans);

      /// \brief Get transform relative to parent
      public: math::Matrix4 GetTransform();

      /// \brief Get transform relative to skeleton origin
      public: math::Matrix4 GetGlobalTransform();

      public: void SetParent(Node* _parent);

      public: Node* GetParent();

      public: bool IsRootNode();

      public: void AddChild(Node* _child);

      public: unsigned int GetChildCount();

      public: Node* GetChild(unsigned int _index);

      /// \brief Get child by name
      public: Node* GetChildByName(std::string _name);

      /// \brief Get child by id
      public: Node* GetChildById(std::string _id);

      /// \brief Get node by name
      public: Node* GetByName(std::string _name);

      /// \brief Get node by id
      public: Node* GetById(std::string _id);

      protected: std::string name;

      protected: std::string id;

      protected: NodeType type;

      protected: math::Matrix4 transform;

      protected: Node *parent;

      protected: std::vector<Node*> children;
    };
    /// \}
  }
}
#endif

