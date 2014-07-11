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
#ifndef _BULLETHINGE2JOINT_HH_
#define _BULLETHINGE2JOINT_HH_

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/Hinge2Joint.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/util/system.hh"

class btHinge2Constraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief A two axis hinge joint
    class GAZEBO_VISIBLE BulletHinge2Joint : public Hinge2Joint<BulletJoint>
    {
      /// \brief Constructor
      public: BulletHinge2Joint(btDynamicsWorld *world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~BulletHinge2Joint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GetAnchor(
                  unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(unsigned int _index,
                  const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GetAxis(
                  unsigned int _index) const;

      // Documentation inherited.
      public: ignition::math::Angle GetAngle(unsigned int _index) const;

      // Documentation inherited.
      public: double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited.
      public: virtual void SetMaxForce(unsigned int _index, double _t);

      // Documentation inherited.
      public: virtual double GetMaxForce(unsigned int _index);

      // Documentation inherited.
      public: virtual bool SetHighStop(unsigned int _index,
                  const ignition::math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(unsigned int _index,
                  const ignition::math::Angle &_angle);

      // Documentation inherited.
      public: virtual ignition::math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual ignition::math::Angle GetLowStop(unsigned int _index);

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GetGlobalAxis(
                  unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle GetAngleImpl(
                  unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _torque);

      /// \brief Pointer to bullet hinge2 constraint
      private: btHinge2Constraint *bulletHinge2;
    };

  /// \}
  }
}
#endif
