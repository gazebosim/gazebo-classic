/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_BULLETUNIVERSALJOINT_HH_
#define _GAZEBO_BULLETUNIVERSALJOINT_HH_

#include <string>
#include "gazebo/physics/bullet/gzBtUniversalConstraint.hh"
#include "gazebo/physics/UniversalJoint.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/util/system.hh"

class btUniversalConstraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief A bullet universal joint class
    class GZ_PHYSICS_VISIBLE BulletUniversalJoint
      : public UniversalJoint<BulletJoint>
    {
      /// \brief Constructor
      public: BulletUniversalJoint(btDynamicsWorld *world, BasePtr _parent);

      /// \brief Destuctor
      public: virtual ~BulletUniversalJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual ignition::math::Vector3d Anchor(
          const unsigned int _index) const;

      // Documentation inherited.
      public: void SetAxis(const unsigned int _index,
          const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetUpperLimit(const unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual void SetLowerLimit(const unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual double UpperLimit(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual double LowerLimit(const unsigned int _index) const;

      // Documentation inherited. \sa Joint::GlobalAxis
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      // Documentation inherited.
      public: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index);

      // Documentation inherited. \sa void BulletJoint::SetForceImpl
      protected: virtual void SetForceImpl(unsigned int _index, double _torque);

      /// \brief Pointer to bullet universal constraint
      private: gzBtUniversalConstraint *bulletUniversal;

      /// \brief Offset angle used in PositionImpl, so that angles are reported
      /// relative to the initial configuration.
      private: double angleOffset[2];

      /// \brief Initial value of joint axis, expressed as unit vector
      /// in world frame.
      private: ignition::math::Vector3d initialWorldAxis[2];
    };
    /// \}
  }
}
#endif
