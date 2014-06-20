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

#ifndef _SIMBODYBALLJOINT_HH_
#define _SIMBODYBALLJOINT_HH_

#include "gazebo/physics/BallJoint.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief SimbodyBallJoint class models a ball joint in Simbody.
    class SimbodyBallJoint : public BallJoint<SimbodyJoint>
    {
      /// \brief Simbody Ball Joint Constructor
      public: SimbodyBallJoint(SimTK::MultibodySystem *_world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyBallJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set joint damping, not yet implemented
      /// \sa Joint::SetDamping
      public: virtual void SetDamping(int _index, double _damping);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetAxis(int /*_index*/) const
              {return math::Vector3();}

      // Documentation inherited.
      public: virtual void SetVelocity(int _index, double _angle);

      // Documentation inherited.
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited.
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited.
      public: virtual void SetMaxForce(int _index, double _t);

      // Documentation inherited.
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(int _index, double _torque);
    };
    /// \}
  }
}
#endif
