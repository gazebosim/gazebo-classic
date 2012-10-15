/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Heightmap shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef HEIGHTMAPSHAPE_HH
#define HEIGHTMAPSHAPE_HH

#include <string>
#include <vector>

#include "common/Image.hh"
#include "math/Vector3.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// HeightmapShape collision shape builds a heightmap from
    /// an image.  The supplied image must be square with
    /// N*N+1 pixels per side, where N is an integer.
    class HeightmapShape : public Shape
    {
      /// \brief Constructor
      public: HeightmapShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~HeightmapShape();

      /// \brief Load the heightmap
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the heightmap
      public: virtual void Init();

      /// \brief Get the URI of the heightmap image.
      /// \return The heightmap image URI.
      public: std::string GetURI() const;

      /// \brief Get the size in meters
      public: math::Vector3 GetSize() const;

      /// \brief Get the origin in world coordinate frame
      public: math::Vector3 GetPos() const;

      /// \brief Return the number of vertices, which equals the size of the
      /// image used to load the heightmap
      /// \return math::Vector2i, result.x = width, result.y = length/height
      public: math::Vector2i GetVertexCount() const;

      /// \brief Get a height at a vertex
      public: float GetHeight(int x, int y);

      public: void FillShapeMsg(msgs::Geometry &_msg);

      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// \brief Get the maximum height
      public: float GetMaxHeight() const;

      /// \brief Get the minimum height
      public: float GetMinHeight() const;

      /// Create a lookup table of the terrain's height
      private: void FillHeightMap();

      protected: std::vector<float> heights;
      protected: common::Image img;

      protected: unsigned int vertSize;
      protected: math::Vector3 scale;
      protected: int subSampling;
    };
    /// \}
  }
}
#endif
