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

#ifndef _SIMBODY_COLLISION_HH_
#define _SIMBODY_COLLISION_HH_

#include <string>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/util/system.hh"

namespace SimTK
{
  class ContactGeometry;
}

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody collisions
    class GAZEBO_VISIBLE SimbodyCollision : public Collision
    {
      /// \brief Constructor
      public: SimbodyCollision(LinkPtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyCollision();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetCategoryBits(unsigned int _bits);

      // Documentation inherited
      public: virtual void SetCollideBits(unsigned int _bits);

      // Documentation inherited
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Set the collision shape.
      /// \param[in] _shape SimTK geometry to use as the collision
      /// SimTK geometry to use as the collision shape.
      public: void SetCollisionShape(SimTK::ContactGeometry *_shape);

      /// \brief Get the simbody collision shape.
      /// \return SimTK geometry used as the collision shape.
      public: SimTK::ContactGeometry *GetCollisionShape() const;

      /// \brief The SimTK collision geometry.
      private: SimTK::ContactGeometry *collisionShape;
    };
    /// \}
  }
}
#endif
