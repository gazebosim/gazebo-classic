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
/* Desc: A universal joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
*/

#ifndef ODEUNIVERSALJOINT_HH
#define ODEUNIVERSALJOINT_HH

#include "physics/UniversalJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief A universal joint
    class ODEUniversalJoint : public UniversalJoint<ODEJoint>
    {
      /// \brief Constructor
      public: ODEUniversalJoint(dWorldID worldId, BasePtr _parent);

      /// \brief Destuctor
      public: virtual ~ODEUniversalJoint();

      /// \brief Get the anchor point
      public: virtual math::Vector3 GetAnchor(int index) const;

      /// \brief Set the anchor point
      public: virtual void SetAnchor(int index, const math::Vector3 &anchor);

      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);

      /// \brief Get the first axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int index) const;

      /// \brief Set the first axis of rotation
      public: virtual void SetAxis(int index, const math::Vector3 &axis);

      /// \brief Get the angle of axis
      public: virtual math::Angle GetAngleImpl(int index) const;

      /// \brief Get the angular rate of an axis
      public: virtual double GetVelocity(int index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);

      /// \brief Set the torque of a joint.
      public: virtual void SetForce(int index, double torque);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);


      /// \brief Set the parameter to value
      public: virtual void SetParam(int parameter, double value);
    };

    /// \}
  }
}
#endif
