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

#ifndef _SIMBODY_RAY_SHAPE_HH_
#define _SIMBODY_RAY_SHAPE_HH_

#include <string>
#include "gazebo/physics/RayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Ray shape for simbody
    class GZ_PHYSICS_VISIBLE SimbodyRayShape : public RayShape
    {
      /// \brief Constructor.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: SimbodyRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor
      /// \param[in] _collision Collision the ray is attached to.
      public: SimbodyRayShape(CollisionPtr _collision);

      /// \brief Destructor
      public: virtual ~SimbodyRayShape();

      // Documentation inherited
      public: virtual void Update();

      // Documentation inherited
      public: virtual void GetIntersection(double &_dist, std::string &_entity);

      // Documentation inherited
      public: virtual void SetPoints(const math::Vector3 &_posStart,
                                     const math::Vector3 &_posEnd);

      /// \brief Pointer to the physics engine.
      private: SimbodyPhysicsPtr physicsEngine;
    };
    /// \}
  }
}
#endif
