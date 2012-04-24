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
/* Desc: Heightmap shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef HEIGHTMAPSHAPE_HH
#define HEIGHTMAPSHAPE_HH

#include <string>
#include <vector>

#include "common/Image.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    /// \brief Height map collision
    class HeightmapShape : public Shape
    {
      /// \brief Constructor
      public: HeightmapShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~HeightmapShape();

      /// \brief Update function
      public: void Update();

      /// \brief Load the heightmap
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the heightmap
      public: virtual void Init();

      public: std::string GetFilename() const;
      public: math::Vector3 GetSize() const;
      public: math::Vector3 GetOrigin() const;

      public: void FillShapeMsg(msgs::Geometry &_msg);

      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      protected: std::vector<double> heights;

      protected: common::Image img;
    };
    /// \}
  }
}
#endif


