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
/* Desc: A ODE hinge joint
 * Author: Nate Koenig
 * Date: 21 May 2003
 */

#ifndef _ODEHINGEJOINT_HH_
#define _ODEHINGEJOINT_HH_

#include "math/Angle.hh"
#include "math/Vector3.hh"

#include "physics/HingeJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief A single axis hinge joint
    class ODEHingeJoint : public HingeJoint<ODEJoint>
    {
      ///  Constructor
      public: ODEHingeJoint(dWorldID worldId, BasePtr _parent);

      /// Destructor
      public: virtual ~ODEHingeJoint();

      /// \brief Load joint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      /// Get the anchor point
      public: virtual math::Vector3 GetAnchor(int index) const;

      /// Set the anchor point
      public: virtual void SetAnchor(int index, const math::Vector3 &anchor);

      /// Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int index) const;

      /// Set the axis of rotation
      public: virtual void SetAxis(int index, const math::Vector3 &axis);

      /// \brief Set the joint damping
      public: virtual void SetDamping(int index, const double damping);

      /// \brief callback to apply damping force to joint
      public: void ApplyDamping();

      /// Get the angle of rotation
      public: virtual math::Angle GetAngleImpl(int index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);

      /// \brief Get the rotation rate of an axis(index)
      public: virtual double GetVelocity(int index) const;

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);

      /// \brief Set the torque of a joint.
      public: virtual void SetForce(int index, double torque);

      /// Get the specified parameter
      public: virtual double GetParam(int parameter) const;

      /// Set the parameter to value
      public: virtual void SetParam(int parameter, double value);

      private: event::ConnectionPtr jointUpdateConnection;
    };

    /// \}
  }
}
#endif
