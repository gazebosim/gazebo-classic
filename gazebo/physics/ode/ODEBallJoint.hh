/*
 * Copyright 2011 Nate Koenig
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
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#ifndef ODEBALLJOINT_HH
#define ODEBALLJOINT_HH

#include "physics/BallJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief An ODEBallJoint
    class ODEBallJoint : public BallJoint<ODEJoint>
    {
      /// \brief Constructor
      public: ODEBallJoint(dWorldID worldId, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~ODEBallJoint();

      /// \brief Get joint's anchor point
      public: virtual math::Vector3 GetAnchor(int index) const;

      /// \brief Set joint's anchor point
      public: virtual void SetAnchor(int index, const math::Vector3 &anchor);

      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int /*index*/) const
              {return math::Vector3();}
      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int /*index*/, double /*angle*/) {}
      /// \brief Get the rotation rate of an axis(index)
      public: virtual double GetVelocity(int /*index*/) const {return 0;}
      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int /*index*/) {return 0;}
      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int /*index*/, double /*t*/) {}
      /// \brief Get the angle of rotation of an axis(index)
      public: virtual math::Angle GetAngleImpl(int /*index*/) const
              {return math::Angle(0);}
    };

    /// \}
  }
}
#endif







