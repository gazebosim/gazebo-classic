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
/* Desc: Occupancy grid collision
 * Author: Nate Koenig
*/

#ifndef _MAPSHAPE_HH_
#define _MAPSHAPE_HH_

#include <deque>
#include <string>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class SpaceTree;
    class QuadNode;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class MapShape MapShape.hh physics/physics.hh
    /// \brief Creates box extrusions based on an image.
    /// This function is not yet complete, to be implemented.
    class GAZEBO_VISIBLE MapShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision object.
      public: explicit MapShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~MapShape();

      /// \brief Update function.
      public: void Update();

      /// \brief Load the map.
      /// \param[in] _sdf Load the map from SDF values.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Init the map.
      public: virtual void Init();

      /// \brief Fills out a msgs::Geometry message containing
      /// information about this map geometry object.
      /// \param[in] _msg Message to fill with this object's data.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief \TODO: Implement this function.
      /// \param[in] _msg Message to process, which will alter the map.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// \brief Returns the image URI for this geometry.
      /// \return The image URI that was used to load the map.
      public: std::string GetURI() const;

      /// \brief Set the scale of the map shape.
      /// \param[in] _scale Scale to set the map shape to.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Returns scaling factor for this geometry.
      /// \return Scaling factor.
      public: virtual math::Vector3 GetScale() const;

      /// \brief Returns image threshold for this geometry.
      /// All regions in image with value larger than MapShape::scale
      /// will be replaced by boxes with MapShape::height.
      /// \return Image threshold value.
      public: int GetThreshold() const;

      /// \brief Returns height of this geometry.  All regions in image with
      /// value larger than MapShape::scale will be replaced by boxes
      /// with MapShape::height.
      /// \return Height of the map shapes.
      public: double GetHeight() const;

      /// \brief Returns granularity of this geometry.
      /// \return Granularity (amount of error betweent the image pixels and
      /// the 3D shapes created).
      public: int GetGranularity() const;

      /// \brief Build the quadtree.
      /// \param[in] _node Quad tree node to build.
      private: void BuildTree(QuadNode *_node);

      /// \brief Get the number of free and occupied pixels in a given area.
      /// \param[in] _xStart X pixel location to start counting.
      /// \param[in] _yStart Y pixel location to start counting.
      /// \param[in] _width Width over which to get the pixel count.
      /// \param[in] _height Height over which to get the pixel count.
      /// \param[out] _freePixels Number of unoccupied pixels.
      /// \param[out] _occPixels Number of occupied pixels.
      private: void GetPixelCount(unsigned int _xStart, unsigned int _yStart,
                                  unsigned int _width, unsigned int _height,
                                  unsigned int &_freePixels,
                                  unsigned int &_occPixels);

      /// \brief Reduce the number of nodes in the tree.
      /// \param[in] _node Quad tree node to reduce.
      private: void ReduceTree(QuadNode *_node);

      /// \brief Try to merge to nodes.
      /// \param[in] _nodeA First quad tree node
      /// \param[in] _nodeB Second quad tree node
      private: void Merge(QuadNode *_nodeA, QuadNode *_nodeB);

      /// \brief Create boxes that represents the map.
      private: void CreateBox();

      /// \brief Create the boxes for the map
      /// \param[in] _node Quad tree node to create boxes from.
      private: void CreateBoxes(QuadNode *_node);

      /// \brief Image used to create the map.
      private: common::Image *mapImage;

      /// \brief Root of the quad tree.
      private: QuadNode *root;

      /// \brief True if the quad tree nodes have been merged.
      private: bool merged;

      /// \brief Counter used to create unique names for each collision
      /// object.
      private: static unsigned int collisionCounter;
    };


    /// \class QuadNode MapShape.hh physics/physics.hh
    /// \cond
    class GAZEBO_VISIBLE QuadNode
    {
      /// \brief Constructor
      /// \param[in] _parent Parent quad tree node.
      public: QuadNode(QuadNode *_parent)
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
    /// \endcond

    /// \}
  }
}
#endif
