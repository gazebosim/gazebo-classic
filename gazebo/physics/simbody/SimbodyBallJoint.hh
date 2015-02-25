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

#ifndef _SIMBODYBALLJOINT_HH_
#define _SIMBODYBALLJOINT_HH_

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
    class GAZEBO_VISIBLE SimbodyBallJoint : public BallJoint<SimbodyJoint>
    {
      /// \brief Simbody Ball Joint Constructor
      public: SimbodyBallJoint(SimTK::MultibodySystem *_world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyBallJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: math::Vector3 GetAnchor(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetAxis(unsigned int /*_index*/) const
              {return math::Vector3();}

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual double GetMaxForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetMaxForce(unsigned int _index, double _t);

      // Documentation inherited.
      public: virtual math::Angle GetAngleImpl(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetGlobalAxis(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(unsigned int _index,
                                   const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(unsigned int _index);

      // Documentation inherited.
      public: virtual bool SetHighStop(unsigned int _index,
                                       const math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(unsigned int _index,
                                      const math::Angle &_angle);

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _torque);
    };
    /// \}
  }
}
#endif
