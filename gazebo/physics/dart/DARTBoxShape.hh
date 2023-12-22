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
#ifndef _GAZEBO_DARTBOXSHAPE_HH_
#define _GAZEBO_DARTBOXSHAPE_HH_

#include "gazebo/physics/BoxShape.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTBoxShapePrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief DART Box shape
    class GZ_PHYSICS_VISIBLE DARTBoxShape : public BoxShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTBoxShape(DARTCollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTBoxShape();

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void SetSize(const ignition::math::Vector3d &_size);

      /// \internal
      /// \brief Pointer to private data
      private: DARTBoxShapePrivate *dataPtr;

      /// \brief True if this link is soft body.
      private: bool isSoftBody;
    };
    /// \}
  }
}
#endif
