/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _SIMBODYHINGEJOINT_HH_
#define _SIMBODYHINGEJOINT_HH_

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/HingeJoint.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief A single axis hinge joint
    class SimbodyHingeJoint : public HingeJoint<SimbodyJoint>
    {
      ///  Constructor
      public: SimbodyHingeJoint(SimTK::MultibodySystem *world, BasePtr _parent);

      /// Destructor
      public: virtual ~SimbodyHingeJoint();

      /// \brief Load the SimbodyHingeJoint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set the axis of rotation
      public: void SetAxis(int _index, const math::Vector3 &_axis);

      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int _index, double _damping);

       /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int _index, double _rate);

      /// \brief Get the rotation rate
      public: virtual double GetVelocity(int _index) const;

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

      /// \brief Set the torque of a joint.
      protected: void SetForceImpl(int _index, double _torque);
    };
    /// \}
  }
}
#endif
