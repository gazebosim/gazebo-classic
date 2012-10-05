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
/* Desc: A ray
 * Author: Nate Koenig, Andrew Howard
 * Date: 14 Oct 2009
 */

#ifndef ODERAYSHAPE_HH
#define ODERAYSHAPE_HH

#include <string>

#include "physics/RayShape.hh"
#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief Ray collision
    class ODERayShape : public RayShape
    {
      /// \brief Constructor for a global ray
      public: ODERayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor
      /// \param body Link the ray is attached to
      public: ODERayShape(CollisionPtr collision);

      /// \brief Destructor
      public: virtual ~ODERayShape();

      /// \brief Update the tay collision
      public: virtual void Update();

      /// \brief Get the nearest intersection
      public: virtual void GetIntersection(double &_dist,
                                           std::string &_entity);

      /// \brief Set the ray based on starting and ending points relative to the
      ///        body
      /// \param posStart Start position, relative the body
      /// \param posEnd End position, relative to the body
      public: virtual void SetPoints(const math::Vector3 &posStart,
                                     const math::Vector3 &posEnd);

      private: static void UpdateCallback(void *data, dGeomID o1, dGeomID o2);

      private: dGeomID geomId;
      private: ODEPhysicsPtr physicsEngine;

      /// \brief an intersection class keeping track of name and depth of
      ///        intersections
      private: class Intersection
               {
                 public: double depth;
                 public: std::string name;
               };
    };
    /// \}
  }
}
#endif






