/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _SIMBODY_MULTIRAYSHAPE_HH_
#define _SIMBODY_MULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Simbody specific version of MultiRayShape
    class GZ_PHYSICS_VISIBLE SimbodyMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor
      public: SimbodyMultiRayShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~SimbodyMultiRayShape();

      // Documentation inherited.
      public: virtual void UpdateRays();

      // Documentation inherited.
      protected: virtual void AddRay(const math::Vector3 &_start,
                             const math::Vector3 &_end);

      /// \brief Pointer to the physics engine.
      private: SimbodyPhysicsPtr physicsEngine;
    };
    /// \}
  }
}
#endif
