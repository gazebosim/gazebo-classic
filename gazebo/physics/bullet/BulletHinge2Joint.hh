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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _BULLETHINGE2JOINT_HH_
#define _BULLETHINGE2JOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/Hinge2Joint.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"

class btHinge2Constraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief A two axis hinge joint
    class BulletHinge2Joint : public Hinge2Joint<BulletJoint>
    {
      /// \brief Constructor
      public: BulletHinge2Joint(btDynamicsWorld *world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~BulletHinge2Joint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      /// \brief Get anchor point
      public: virtual math::Vector3 GetAnchor(int _index) const;

      /// \brief Set the first axis of rotation
      public: virtual void SetAxis(int _index, const math::Vector3 &_axis);

      /// \brief Get first axis of rotation
      public: virtual math::Vector3 GetAxis(int _index) const;

      /// \brief Get angle of rotation about first axis
      public: math::Angle GetAngle(int _index) const;

      /// \brief Get rate of rotation about first axis
      public: double GetVelocity(int _index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int _index, double _angle);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int _index, double _t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int _index);

      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      /// \brief Get the high stop of an axis(index).
      public: virtual math::Angle GetHighStop(int _index);

      /// \brief Get the low stop of an axis(index).
      public: virtual math::Angle GetLowStop(int _index);

      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      /// \brief Get the angle of rotation
      public: virtual math::Angle GetAngleImpl(int _index) const;

      /// \brief Set the torque
      protected: virtual void SetForceImpl(int _index, double _torque);

      /// \brief Pointer to bullet hinge2 constraint
      private: btHinge2Constraint *bulletHinge2;
    };

  /// \}
  }
}
#endif
