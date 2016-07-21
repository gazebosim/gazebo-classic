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
#ifndef _GAZEBO_PHYSICS_ODE_ODESLIDERJOINT_HH_
#define _GAZEBO_PHYSICS_ODE_ODESLIDERJOINT_HH_

#include "gazebo/physics/SliderJoint.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief A slider joint
    class GZ_PHYSICS_VISIBLE ODESliderJoint : public SliderJoint<ODEJoint>
    {
      /// \brief Constructor
      /// \param[in] _worldId ODE world id.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: ODESliderJoint(dWorldID _worldId, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~ODESliderJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual ignition::math::Vector3d Anchor(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(const unsigned int _index,
                                     const ignition::math::Vector3d &_anchor);

      // Documentation inherited
      public: virtual ignition::math::Vector3d GlobalAxis(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(const unsigned int _index,
                  const ignition::math::Vector3d &_axis);

      // Documentation inherited
      public: virtual ignition::math::Angle AngleImpl(
                  const unsigned int _index) const;

      // Documentation inherited
      public: virtual double Velocity(const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(const unsigned int _index,
                  const double _angle);

      // Documentation inherited
      public: virtual double Param(const unsigned int _parameter) const;

      // Documentation inherited
      public: virtual void SetParam(const unsigned int _parameter,
                  const double _value);

      // Documentation inherited
      protected: virtual void SetForceImpl(const unsigned int _index,
                     const double _effort);
    };
    /// \}
  }
}
#endif
