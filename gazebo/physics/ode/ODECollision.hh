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
#ifndef _GAZEBO_PHYSICS_ODECOLLISION_HH_
#define _GAZEBO_PHYSICS_ODECOLLISION_HH_

#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    // Forward declare private data class.
    class ODECollisionPrivate;

    /// \brief Base class for all ODE collisions.
    class GZ_PHYSICS_VISIBLE ODECollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit ODECollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~ODECollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Set the encapsulated collsion object.
      /// \param[in] _collisionId ODE id of the collision object.
      /// \param[in] _placeable True to make the object movable.
      public: void SetCollision(dGeomID _collisionId, const bool _placeable);

      /// \brief Return the collision id.
      /// \return The collision id.
      /// \deprecated See CollisionId()
      public: dGeomID GetCollisionId() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return the collision id.
      /// \return The collision id.
      public: dGeomID CollisionId() const;

      /// \brief Get the ODE collision class.
      /// \return The ODE collision class.
      /// \deprecated See CollisionClass()
      public: int GetCollisionClass() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the ODE collision class.
      /// \return The ODE collision class.
      public: int CollisionClass() const;

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(const unsigned int bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(const unsigned int bits);

      // Documentation inherited.
      public: virtual ignition::math::Box BoundingBox() const;

      /// \brief Get the collision's space ID
      /// \return The collision's space ID
      /// \deprecated See SpaceId()
      public: dSpaceID GetSpaceId() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the collision's space ID
      /// \return The collision's space ID
      public: dSpaceID SpaceId() const;

      /// \brief Set the collision's space ID
      /// \param[in] _spaceid ID of an ODE collision space.
      public: void SetSpaceId(const dSpaceID _spaceid);

      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to ODESurfaceParams.
      /// \return Dynamically casted pointer to ODESurfaceParams.
      /// \deprecated See ODESurface()
      public: ODESurfaceParamsPtr GetODESurface() const GAZEBO_DEPRECATED(7.0);

      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to ODESurfaceParams.
      /// \return Dynamically casted pointer to ODESurfaceParams.
      public: ODESurfaceParamsPtr ODESurface() const;

      /// \brief Used when this is static to set the posse.
      private: void OnPoseChangeGlobal();

      /// \brief Used when this is not statis to set the posse.
      private: void OnPoseChangeRelative();

      /// \brief Empty pose change callback.
      private: void OnPoseChangeNull();

      /// \internal
      /// \brief Private data pointer.
      protected: ODECollisionPrivate *odeCollisionDPtr;
    };
    /// \}
  }
}
#endif
