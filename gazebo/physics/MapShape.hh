/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_MAPSHAPE_HH_
#define _GAZEBO_PHYSICS_MAPSHAPE_HH_

#include <deque>
#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Image;
  }

  namespace physics
  {
    class QuadNode;

    // Forward declare private data class
    class MapShapePrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class MapShape MapShape.hh physics/physics.hh
    /// \brief Creates box extrusions based on an image.
    /// This function is not yet complete, to be implemented.
    class GZ_PHYSICS_VISIBLE MapShape : public Shape
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
      /// \deprecated See URI() const
      public: std::string GetURI() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the image URI for this geometry.
      /// \return The image URI that was used to load the map.
      public: std::string URI() const;

      /// \brief Set the scale of the map shape.
      /// \param[in] _scale Scale to set the map shape to.
      /// \deprecated See function that accepts ignition::math parameters.
      public: void SetScale(const math::Vector3 &_scale) GAZEBO_DEPRECATED(7.0);

      /// \brief Set the scale of the map shape.
      /// \param[in] _scale Scale to set the map shape to.
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Returns scaling factor for this geometry.
      /// \return Scaling factor.
      /// \deprecated See function that returns an ignition::math object
      public: virtual math::Vector3 GetScale() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns scaling factor for this geometry.
      /// \return Scaling factor.
      public: virtual ignition::math::Vector3d Scale() const;

      /// \brief Returns image threshold for this geometry.
      /// All regions in image with value larger than MapShape::scale
      /// will be replaced by boxes with MapShape::height.
      /// \return Image threshold value.
      /// \deprecated See Threshold() const
      public: int GetThreshold() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns image threshold for this geometry.
      /// All regions in image with value larger than MapShape::scale
      /// will be replaced by boxes with MapShape::height.
      /// \return Image threshold value.
      public: int Threshold() const;

      /// \brief Returns height of this geometry.  All regions in image with
      /// value larger than MapShape::scale will be replaced by boxes
      /// with MapShape::height.
      /// \return Height of the map shapes.
      /// \deprecated See Height() const
      public: double GetHeight() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns height of this geometry.  All regions in image with
      /// value larger than MapShape::scale will be replaced by boxes
      /// with MapShape::height.
      /// \return Height of the map shapes.
      public: double Height() const;

      /// \brief Returns granularity of this geometry.
      /// \return Granularity (amount of error betweent the image pixels and
      /// the 3D shapes created).
      /// \deprecated See Granularity() const
      public: int GetGranularity() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns granularity of this geometry.
      /// \return Granularity (amount of error betweent the image pixels and
      /// the 3D shapes created).
      public: int Granularity() const;

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
      private: void PixelCount(const unsigned int _xStart,
                   const unsigned int _yStart,
                   const unsigned int _width,
                   const unsigned int _height,
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

      /// \internal
      /// \brief Private data pointer
      private: MapShapePrivate *mapShapeDPtr;
    };
    /// \}
  }
}
#endif
