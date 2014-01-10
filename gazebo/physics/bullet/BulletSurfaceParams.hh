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

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Bullet surface parameters.
    class BulletSurfaceParams : public SurfaceParams
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

      /// \brief Dry friction coefficient in the primary friction direction
      ///        as defined by the friction pyramid.  This is fdir1 if defined,
      ///        otherwise, a vector consstrained to be perpendicular to the
      ///        contact normal in the global y-z plane is used.
      /// \sa    http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double mu1;

      /// \brief Dry friction coefficient in the second friction direction
      ///        as defined by the friction pyramid.  This is fdir1 if defined,
      ///        otherwise, a vector consstrained to be perpendicular to the
      ///        contact normal in the global y-z plane is used.
      /// \sa    http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double mu2;
    };
    /// \}
  }
}
#endif
