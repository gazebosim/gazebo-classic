/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_DARTPLANESHAPE_HH_
#define _GAZEBO_DARTPLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTPlaneShapePrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief An DART Plane shape.
    class GZ_PHYSICS_VISIBLE DARTPlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      /// \deprecated See version that accepts DARTCollisionPtr
      public: explicit DARTPlaneShape(CollisionPtr _parent)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTPlaneShape(DARTCollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTPlaneShape();

      // Documentation inherited
      public: virtual void CreatePlane();

      // Documentation inherited
      public: virtual void SetAltitude(const math::Vector3 &_pos);

      /// \internal
      /// \brief Pointer to private data
      private: DARTPlaneShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
