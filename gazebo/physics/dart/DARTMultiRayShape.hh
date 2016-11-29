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

#ifndef _GAZEBO_DARTMULTIRAYSHAPE_HH_
#define _GAZEBO_DARTMULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_dart
    /// \{

    /// Forward declare private data class
    class DARTMultiRayShapePrivate;

    /// \brief DART specific version of MultiRayShape
    class GZ_PHYSICS_VISIBLE DARTMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      /// \deprecated See version that accepts DARTCollisionPtr
      public: explicit DARTMultiRayShape(CollisionPtr _parent)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTMultiRayShape(DARTCollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTMultiRayShape();

      // Documentation inherited.
      public: virtual void UpdateRays();

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start location of the ray.
      /// \param[in] _end End location of the ray.
      protected: void AddRay(const math::Vector3 &_start,
                             const math::Vector3 &_end);

      /// \internal
      /// \brief Pointer to private data
      private: DARTMultiRayShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
