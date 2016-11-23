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

#ifndef _GAZEBO_DARTCYLINDERSHAPE_HH_
#define _GAZEBO_DARTCYLINDERSHAPE_HH_

#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTCylinderShapePrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief DART cylinder shape
    class GZ_PHYSICS_VISIBLE DARTCylinderShape : public CylinderShape
    {
      /// \brief Constructor
      /// \param[in] _parent Collision parent.
      /// \deprecated See version that accepts DARTCollisionPtr.
      public: explicit DARTCylinderShape(CollisionPtr _parent)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _parent Collision parent.
      public: explicit DARTCylinderShape(DARTCollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTCylinderShape();

      // Documentation inerited.
      public: void SetSize(double _radius, double _length);

      /// \internal
      /// \brief Pointer to private data
      private: DARTCylinderShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
