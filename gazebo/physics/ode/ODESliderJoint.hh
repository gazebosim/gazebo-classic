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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef ODESLIDERJOINT_HH
#define ODESLIDERJOINT_HH

#include "physics/SliderJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief A slider joint
    class ODESliderJoint : public SliderJoint<ODEJoint>
    {
      /// \brief Constructor
      public: ODESliderJoint(dWorldID worldId, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~ODESliderJoint();

      /// \brief Load the ODESliderJoint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int index) const;

      /// \brief Set the axis of motion
      public: virtual void SetAxis(int index, const math::Vector3 &axis);

      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);

      /// \brief callback to apply damping force to joint
      public: void ApplyDamping();

      /// \brief Get the position of the joint
      public: virtual math::Angle GetAngleImpl(int index) const;

      /// \brief Get the rate of change
      public: virtual double GetVelocity(int index) const;

      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);

      /// \brief Set the slider force
      public: virtual void SetForce(int index, double force);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);

      /// \brief Get the _parameter
      public: virtual double GetParam(int parameter) const;

      /// \brief Set the _parameter
      public: virtual void SetParam(int parameter, double value);
    };

    /// \}
  }
}
#endif






