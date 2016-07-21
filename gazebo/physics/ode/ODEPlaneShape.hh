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
#ifndef GAZEBO_PHYSICS_ODE_ODEPLANESHAPE_HH_
#define GAZEBO_PHYSICS_ODE_ODEPLANESHAPE_HH_
#include <ignition/math/Vector3.hh>

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief An ODE Plane shape.
    class GZ_PHYSICS_VISIBLE ODEPlaneShape : public PlaneShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit ODEPlaneShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~ODEPlaneShape() = default;

      // Documentation inherited
      public: virtual void CreatePlane();

      // Documentation inherited
      public: virtual void SetAltitude(const ignition::math::Vector3d &_pos);
    };
    /// \}
  }
}
#endif
