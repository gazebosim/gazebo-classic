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
#ifndef GAZEBO_PHYSICS_MAPSHAPEPRIVATE_HH_
#define GAZEBO_PHYSICS_MAPSHAPEPRIVATE_HH_

#include <string>
#include <deque>

#include "gazebo/common/Image.hh"
#include "gazebo/physics/ShapePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Quad tree node used by MapShape
    class QuadNode
    {
      /// \brief Constructor
      /// \param[in] _parent Parent quad tree node.
      public: explicit QuadNode(QuadNode *_parent)
              : x(0), y(0), width(0), height(0)
              {
                parent = _parent;
                occupied = false;
                leaf = true;
                valid = true;
              }

      /// \brief Destructor.
      public: ~QuadNode()
              {
                std::deque<QuadNode*>::iterator iter;
                for (iter = children.begin(); iter != children.end(); ++iter)
                    delete (*iter);
              }

      /// \brief Print this quad tree node, and all its children.
      /// \param[in] _space String of spaces that formats the printfs.
      public: void Print(std::string _space)
              {
                std::deque<QuadNode*>::iterator iter;

                printf("%sXY[%d %d] WH[%d %d] O[%d] L[%d] V[%d]\n",
                    _space.c_str(), x, y, width, height, occupied, leaf, valid);
                _space += "  ";
                for (iter = children.begin(); iter != children.end(); ++iter)
                  if ((*iter)->occupied)
                    (*iter)->Print(_space);
              }

      /// \brief X and Y location of the node.
      public: uint32_t x, y;

      /// \brief Width and height of the node.
      public: uint32_t width, height;

      /// \brief Parent node.
      public: QuadNode *parent;

      /// \brief Children nodes.
      public: std::deque<QuadNode*> children;

      /// \brief True if the node is occupied
      public: bool occupied;

      /// \brief True if the node is a leaf.
      public: bool leaf;

      /// \brief True if the node is valid.
      public: bool valid;
    };

    /// \internal
    /// \brief Private data for MapShape
    class MapShapePrivate : public ShapePrivate
    {
      /// \brief Image used to create the map.
      public: common::Image *mapImage;

      /// \brief Root of the quad tree.
      public: QuadNode *root;

      /// \brief True if the quad tree nodes have been merged.
      public: bool merged;

      /// \brief Counter used to create unique names for each collision
      /// object.
      public: static unsigned int collisionCounter;
    };
  }
}
#endif
