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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef ODECOLLISION_HH
#define ODECOLLISION_HH

#include "ode/ode.h"

#include "common/CommonTypes.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/Collision.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief Base class for all ODE collisions
    class ODECollision : public Collision
    {
      /// \brief Constructor
      public: ODECollision(LinkPtr link);

      /// \brief Destructor
      public: virtual ~ODECollision();

      /// \brief Load the collision
      public: virtual void Load(sdf::ElementPtr _sdf);

      /*public: virtual void Load()
              {
                Base::Load();
              }
              */

      /// \brief Finalize the collision
      public: void Fini();

      /// \brief Set the encapsulated geometry object
      public: void SetCollision(dGeomID collisionId, bool placeable);

      /// \brief Return the collision id
      /// \return The collision id
      public: dGeomID GetCollisionId() const;

      /// \brief Get the ODE collision class
      public: int GetCollisionClass() const;

      public: virtual void OnPoseChange();

      /// \brief Set the category bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCategoryBits(unsigned int bits);

      /// \brief Set the collide bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCollideBits(unsigned int bits);

      /// \brief Get the bounding box, defined by the physics engine
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get the collision's space ID
      public: dSpaceID GetSpaceId() const;

      /// \brief Set the collision's space ID
      public: void SetSpaceId(dSpaceID spaceid);

      private: void OnPoseChangeGlobal();
      private: void OnPoseChangeRelative();
      private: void OnPoseChangeNull();

      protected: dSpaceID spaceId;

      ///  ID for the sub-collision
      protected: dGeomID collisionId;

      private: void (ODECollision::*onPoseChangeFunc)();
    };

    /// \}
  }
}
#endif
