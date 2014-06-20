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

#ifndef _SIMBODY_HINGEJOINT_HH_
#define _SIMBODY_HINGEJOINT_HH_

#include <vector>

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

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void SetDamping(int _index, double _damping);

      // Documentation inherited.
      public: void SetAxis(int _index, const math::Vector3 &_axis);

      // Documentation inherited.
      public: virtual void SetVelocity(int _index, double _rate);

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

      /// \brief save simbody state for spawning
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      /// \brief restore  simbody state for spawning
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      // Documentation inherited.
      protected: virtual math::Angle GetAngleImpl(int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(int _index, double _torque);

      /// \brief save simbody state for reconstructing simbody model graph
      private: std::vector<double> simbodyQ;

      /// \brief save simbody state for reconstructing simbody model graph
      private: std::vector<double> simbodyU;
    };
    /// \}
  }
}
#endif
