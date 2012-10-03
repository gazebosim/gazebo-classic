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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef ODEHINGE2JOINT_HH
#define ODEHINGE2JOINT_HH

#include "math/Angle.hh"
#include "math/Vector3.hh"

#include "physics/Hinge2Joint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief A two axis hinge joint
    class ODEHinge2Joint : public Hinge2Joint<ODEJoint>
    {
      /// \brief Constructor
      public: ODEHinge2Joint(dWorldID worldId, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~ODEHinge2Joint();

      /// \brief Load the ODEHinge2Joint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set the anchor point
      public: virtual void SetAnchor(int index, const math::Vector3 &anchor);

      /// \brief Set the first axis of rotation
      public: virtual void SetAxis(int index, const math::Vector3 &axis);

      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);

      /// \brief Get anchor point
      public: virtual math::Vector3 GetAnchor(int index) const;

      /// \brief Get first axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int index) const;

      /// \brief Get angle of rotation about first axis
      public: virtual math::Angle GetAngleImpl(int index) const;

      /// \brief Get rate of rotation about first axis
      public: virtual double GetVelocity(int index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);

      /// \brief Set the torque
      public: virtual void SetForce(int index, double torque);

      /// \brief Get the specified parameter
      public: virtual double GetParam(int parameter) const;

      /// \brief Set _parameter with _value
      public: virtual void SetParam(int parameter, double value);
    };

    /// \}
  }
}
#endif







