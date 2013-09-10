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
/* Desc: A screw or primastic joint
 * Author: Nate Koenig
 * Date: 24 May 2009
 */

#ifndef _SIMBODYSCREWJOINT_HH_
#define _SIMBODYSCREWJOINT_HH_

#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief A screw joint
    class SimbodyScrewJoint : public ScrewJoint<SimbodyJoint>
    {
      /// \brief Constructor
      public: SimbodyScrewJoint(SimTK::MultibodySystem *world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyScrewJoint();

      /// \brief Load the SimbodyScrewJoint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int _index, double _damping);

      // Documentation inherited.
      public: virtual void Init();

      /// \brief Set the axis of motion
      public: void SetAxis(int _index, const math::Vector3 &_axis);


      /// \copydoc ScrewJoint::SetThreadPitch
      public: virtual void SetThreadPitch(int _index, double _threadPitch);

      /// \copydoc ScrewJoint::GetThreadPitch
      public: virtual double GetThreadPitch(unsigned int /*_index*/)
              {
                gzerr << "Not implemented in Simbody\n";
                return 0;
              }

      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int _index, const math::Angle &_angle);

      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int _index, const math::Angle &_angle);

      /// \brief Get the high stop of an axis(index).
      public: virtual math::Angle GetHighStop(int _index);

      /// \brief Get the low stop of an axis(index).
      public: virtual math::Angle GetLowStop(int _index);

      /// \brief Get the rate of change
      public: virtual double GetVelocity(int _index) const;

       /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int _index, double _angle);

      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int _index, double _t);

      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int _index);

      /// \brief Get the axis of rotation
      public: virtual math::Vector3 GetGlobalAxis(int _index) const;

      /// \brief Get the angle of rotation
      public: virtual math::Angle GetAngleImpl(int _index) const;

      /// \brief Set the screw force
      protected: virtual void SetForceImpl(int _index, double _force);
    };
    /// \}
  }
}
#endif
