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
#ifndef _GAZEBO_PHYSICS_SIMBODY_SIMBODYBALLJOINT_HH_
#define _GAZEBO_PHYSICS_SIMBODY_SIMBODYBALLJOINT_HH_

#include "gazebo/physics/BallJoint.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief SimbodyBallJoint class models a ball joint in Simbody.
    class GZ_PHYSICS_VISIBLE SimbodyBallJoint : public BallJoint<SimbodyJoint>
    {
      /// \brief Simbody Ball Joint Constructor
      public: SimbodyBallJoint(SimTK::MultibodySystem *_world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyBallJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: ignition::math::Vector3d Anchor(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d Axis(
                  const unsigned int /*_index*/) const;

      // Documentation inherited.
      public: virtual void SetVelocity(const unsigned int _index,
                  const double _angle);

      // Documentation inherited.
      public: virtual double Velocity(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle AngleImpl(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GlobalAxis(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(const unsigned int _index,
                                   const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual ignition::math::Angle HighStop(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle LowStop(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetHighStop(const unsigned int _index,
                                       const ignition::math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(const unsigned int _index,
                                      const math::Angle &_angle);

      // Documentation inherited.
      protected: virtual void SetForceImpl(
                     const unsigned int _index, const double _torque);
    };
    /// \}
  }
}
#endif
