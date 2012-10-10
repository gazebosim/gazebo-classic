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
/* Desc: Plane shape
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#ifndef PLANESHAPE_HH
#define PLANESHAPE_HH

#include "common/CommonTypes.hh"
#include "Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Collision for an infinite plane.
    /// This collision is used primarily for ground planes.  Note that while
    /// the plane in infinite, only the part near the camera is drawn.
    class PlaneShape : public Shape
    {
      /// \brief Constructor
      /// \param body Link to which we are attached.
      public: PlaneShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~PlaneShape();

      /// \brief Initialize the plane
      public: virtual void Init();

      /// \brief Create the plane
      public: virtual void CreatePlane();

      /// \brief Set the altitude of the plane
      public: virtual void SetAltitude(const math::Vector3 &_pos);

      /// \brief Set the normal
      public: void SetNormal(const math::Vector3 &_norm);

      public: math::Vector3 GetNormal() const;

      /// \brief Set the size
      public: void SetSize(const math::Vector2d &_size);

      /// \brief Get the size
      public: math::Vector2d GetSize() const;

      public: void FillShapeMsg(msgs::Geometry &_msg);

      public: virtual void ProcessMsg(const msgs::Geometry &_msg);
    };
    /// \}
  }
}
#endif


