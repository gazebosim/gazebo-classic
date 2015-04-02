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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _BULLETBALLJOINT_HH_
#define _BULLETBALLJOINT_HH_

#include "gazebo/physics/BallJoint.hh"
#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/util/system.hh"

class btPoint2PointConstraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief BulletBallJoint class models a ball joint in Bullet.
    class GAZEBO_VISIBLE BulletBallJoint : public BallJoint<BulletJoint>
    {
      /// \brief Bullet Ball Joint Constructor
      public: BulletBallJoint(btDynamicsWorld *_world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~BulletBallJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: math::Vector3 GetAnchor(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetAxis(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual double GetMaxForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetMaxForce(unsigned int _index, double _t);

      // Documentation inherited.
      public: virtual bool SetHighStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(unsigned int _index,
                  const math::Angle &_angle);

      // Documentation inherited.
      public: virtual math::Angle GetAngleImpl(unsigned int _index) const;

      // Documentation inherited.
      public: virtual math::Vector3 GetGlobalAxis(unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _torque);

      // Documentation inherited.
      public: virtual void SetAxis(unsigned int _index,
                                   const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual math::Angle GetHighStop(unsigned int _index);

      // Documentation inherited.
      public: virtual math::Angle GetLowStop(unsigned int _index);

      /// \brief bullet ball constraint
      private: btPoint2PointConstraint *bulletBall;
    };

    /// \}
  }
}
#endif
