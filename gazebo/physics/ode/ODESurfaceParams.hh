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

#ifndef _ODESURFACEPARAMS_HH_
#define _ODESURFACEPARAMS_HH_

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

    /// \brief ODE surface parameters.
    class GZ_PHYSICS_ODE_VISIBLE ODESurfaceParams : public SurfaceParams
    {
      /// \brief Constructor.
      public: ODESurfaceParams();

      /// \brief Destructor.
      public: virtual ~ODESurfaceParams();

      /// \brief Load the contact params.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void FillMsg(msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual void ProcessMsg(const msgs::Surface &_msg);

      // Documentation inherited.
      public: virtual FrictionPyramidPtr GetFrictionPyramid() const;

      /// \brief bounce restitution coefficient [0,1], with 0 being inelastic,
      ///        and 1 being perfectly elastic.
      /// \sa    http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double bounce;

      /// \brief minimum contact velocity for bounce to take effect, otherwise
      ///        the collision is treated as an inelastic collision.
      /// \sa    http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double bounceThreshold;

      /// \brief spring constant equivalents of a contact as a function of
      ///        SurfaceParams::cfm and SurfaceParams::erp.
      /// \sa    See for example
      ///        http://www.ode.org/ode-latest-userguide.html#sec_3_8_2
      ///        for more details.
      public: double kp;

      /// \brief spring damping constant equivalents of a contact as a
      ///        function of SurfaceParams::cfm and SurfaceParams::erp.
      /// \sa    See for example
      ///        http://www.ode.org/ode-latest-userguide.html#sec_3_8_2
      ///        for more details.
      public: double kd;

      /// \brief Constraint Force Mixing parameter.
      ///        See for example
      ///        http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
      ///        for more details.
      public: double cfm;

      /// \brief Error Reduction Parameter.
      /// \sa    See for example
      ///        http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
      ///        for more details.
      public: double erp;

      /// \brief Maximum interpenetration error correction velocity.  If
      ///        set to 0, two objects interpenetrating each other
      ///        will not be pushed apart.
      /// \sa    See dWroldSetContactMaxCorrectingVel
      ///        (http://www.ode.org/ode-latest-userguide.html#sec_5_2_0)
      public: double maxVel;

      /// \brief Minimum depth before ERP takes effect.
      /// \sa    See dWorldSetContactSurfaceLayer
      ///        (http://www.ode.org/ode-latest-userguide.html#sec_5_2_0)
      public: double minDepth;

      /// \brief Artificial contact slip in the primary friction direction.
      /// \sa    See dContactSlip1 in
      ///        http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double slip1;

      /// \brief Artificial contact slip in the secondary friction dirction.
      /// \sa    See dContactSlip2 in
      ///        http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: double slip2;

      public: double elasticModulus;

      public: double elasticModulusReferenceLength;

      /// \brief Friction pyramid parameters (mu1, mu2).
      /// Note that the primary friction pyramid direction can be specified
      /// by fdir1, otherwise a vector constrained to be perpendicular to the
      /// contact normal in the global y-z plane is used.
      /// \sa    http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      private: FrictionPyramidPtr frictionPyramid;
    };
    /// \}
  }
}
#endif
