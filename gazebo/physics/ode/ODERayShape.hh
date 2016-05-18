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
/* Desc: A ray
 * Author: Nate Koenig, Andrew Howard
 * Date: 14 Oct 2009
 */

#ifndef _ODERAYSHAPE_HH_
#define _ODERAYSHAPE_HH_

#include <string>

#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief Ray collision
    class GZ_PHYSICS_VISIBLE ODERayShape : public RayShape
    {
      /// \brief Constructor for a global ray.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit ODERayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor for a ray in a space.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      /// \param[in] _spaceID Space id.
      public: ODERayShape(PhysicsEnginePtr _physicsEngine, dSpaceID _spaceId);

      /// \brief Constructor.
      /// \param[in] _collision Collision object this ray is attached to.
      public: explicit ODERayShape(CollisionPtr _collision);

      /// \brief Destructor.
      public: virtual ~ODERayShape();

      /// \brief Update the ray collision
      public: virtual void Update();

      /// \brief Get the nearest intersection
      /// \param[out] _dist Distance to the intersection.
      /// \param[out] _entity Name of the entity that was hit.
      public: virtual void GetIntersection(double &_dist, std::string &_entity);

      /// \brief Set the ray based on starting and ending points relative to
      ///        the body
      /// \param[in] _posStart Start position, relative the body
      /// \param[in] _posEnd End position, relative to the body
      public: virtual void SetPoints(const math::Vector3 &_posStart,
                                     const math::Vector3 &_posEnd);

      /// \brief Get the ODE Geom ID
      /// \return The ODE Geom id
      public: dGeomID ODEGeomId() const;

      /// \brief Ray-intersection callback.
      /// \param[in] _data Pointer to user data.
      /// \param[in] _o1 First geom to check for collisions.
      /// \param[in] _o2 Second geom to check for collisions.
      private: static void UpdateCallback(void *_data, dGeomID _o1,
                                          dGeomID _o2);

      /// \brief ODE geom id.
      private: dGeomID geomId;

      /// \brief Pointer to the ODE physics engine
      private: ODEPhysicsPtr physicsEngine;

      /// \brief An intersection class keeping track of name and depth of
      ///        intersections.
      private: class Intersection
               {
                 /// \brief Depth of the ray intersection.
                 public: double depth;

                 /// \brief Name of the collision object that was hit.
                 public: std::string name;
               };
    };
    /// \}
  }
}
#endif
