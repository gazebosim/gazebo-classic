/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef GAZEBO_PHYSICS_DART_DARTCOLLISION_HH_
#define GAZEBO_PHYSICS_DART_DARTCOLLISION_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTCollisionPrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief Base class for all DART collisions.
    class GZ_PHYSICS_VISIBLE DARTCollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit DARTCollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTCollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(unsigned int _bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(unsigned int _bits);

      /// \brief Get the category bits, used during collision detection
      /// \return The bits
      public: virtual unsigned int GetCategoryBits() const;

      /// \brief Get the collide bits, used during collision detection
      /// \return The bits
      public: virtual unsigned int GetCollideBits() const;

      // Documentation inherited.
      public: virtual ignition::math::Box BoundingBox() const;

      /// \brief Get DART body node.
      /// \return Pointer to the dart BodyNode.
      public: dart::dynamics::BodyNode *DARTBodyNode() const;

      /// \brief Set DART collision shape.
      /// \param[in] _shape DART Collision shape
      /// \param[in] _placeable True to make the object movable.
      public: void SetDARTCollisionShapeNode(
                           dart::dynamics::ShapeNodePtr _shape,
                           const bool _placeable = true);

      /// \brief Get DART collision shape node.
      /// \return DART Collision shape pointer.
      public: dart::dynamics::ShapeNodePtr DARTCollisionShapeNode() const;


      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to DARTSurfaceParams.
      /// \return Dynamically casted pointer to DARTSurfaceParams.
      public: DARTSurfaceParamsPtr DARTSurface() const;

      /// \internal
      /// \brief Pointer to private data
      private: DARTCollisionPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
