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

#ifndef _BULLETSURFACEPARAMS_HH_
#define _BULLETSURFACEPARAMS_HH_

#include <sdf/sdf.hh>

#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Bullet surface parameters.
    class GAZEBO_VISIBLE BulletSurfaceParams : public SurfaceParams
    {
      /// \brief Constructor.
      public: BulletSurfaceParams();

      /// \brief Destructor.
      public: virtual ~BulletSurfaceParams();

      /// \brief Load the contact params.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void FillMsg(msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual void ProcessMsg(const msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual FrictionPyramidPtr GetFrictionPyramid() const;

      /// \brief Friction pyramid parameters (mu1, mu2).
      // For further documentation, see:
      // github.com/erwincoumans/bullet3/blob/master/src/BulletDynamics
      // /ConstraintSolver/btSequentialImpulseConstraintSolver.cpp#L910
      private: FrictionPyramidPtr frictionPyramid;
    };
    /// \}
  }
}
#endif
