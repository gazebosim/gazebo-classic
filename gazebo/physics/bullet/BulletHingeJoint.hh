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
/* Desc: A body that has a box shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _BULLETHINGEJOINT_HH_
#define _BULLETHINGEJOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/HingeJoint.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"

class btHingeConstraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief A single axis hinge joint
    class BulletHingeJoint : public HingeJoint<BulletJoint>
    {
      ///  Constructor
      public: BulletHingeJoint(btDynamicsWorld *world, BasePtr _parent);

      /// Destructor
      public: virtual ~BulletHingeJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual math::Vector3 GetAnchor(int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual void SetVelocity(int _index, double _vel);

      // Documentation inherited.
      public: virtual double GetVelocity(int _index) const;

      // Documentation inherited.
      public: virtual void SetMaxForce(int _index, double _t);

      // Documentation inherited.
      public: virtual double GetMaxForce(int _index);

      // Documentation inherited.
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(int _index);

      // Documentation inherited.
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      // Documentation inherited.
      public: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(int _index, double _effort);

      /// \brief Pointer to bullet hinge constraint.
      private: btHingeConstraint *bulletHinge;

      /// \brief Offset angle used in GetAngleImpl, so that angles are reported
      ///        relative to the initial configuration.
      private: double angleOffset;

      /// \brief Initial value of joint axis, expressed as unit vector
      ///        in world frame.
      private: math::Vector3 initialWorldAxis;
    };
    /// \}
  }
}
#endif
