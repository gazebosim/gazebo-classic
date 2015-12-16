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
#ifndef _GAZEBO_PHYSICS_INERTIAL_PRIVATE_HH_
#define _GAZEBO_PHYSICS_INERTIAL_PRIVATE_HH_

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private Inertial data
    class InertialPrivate
    {
      /// \brief Mass the object. Default is 1.0.
      public: double mass;

      /// \brief Center of gravity in the Link frame.
      /// Default is (0.0 0.0 0.0  0.0 0.0 0.0)
      public: ignition::math::Pose3d cog;

      /// \brief Principal moments of inertia. Default is (1.0 1.0 1.0)
      /// These Moments of Inertia are specified in the local Inertial frame.
      public: ignition::math::Vector3d principals;

      /// \brief Product moments of inertia. Default is (0.0 0.0 0.0)
      /// These MOI off-diagonals are specified in the local Inertial frame.
      /// Where products.x is Ixy, products.y is Ixz and products.z is Iyz.
      public: ignition::math::Vector3d products;

      /// \brief Our SDF values.
      public: sdf::ElementPtr sdf;

      /// \brief An SDF pointer that allows us to only read the inertial.sdf
      /// file once, which in turns limits disk reads.
      public: static sdf::ElementPtr sdfInertial;
    };
  }
}
#endif
