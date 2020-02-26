/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/physics/HingeJoint.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief A single axis hinge joint
    class GZ_PHYSICS_VISIBLE SimbodyHingeJoint : public HingeJoint<SimbodyJoint>
    {
      ///  Constructor
      public: SimbodyHingeJoint(SimTK::MultibodySystem *world, BasePtr _parent);

      /// Destructor
      public: virtual ~SimbodyHingeJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: void SetAxis(const unsigned int _index,
          const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _rate);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      /// \brief save simbody state for spawning
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      /// \brief restore  simbody state for spawning
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      // Documentation inherited.
      protected: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _torque);

      /// \brief save simbody state for reconstructing simbody model graph
      private: std::vector<double> simbodyQ;

      /// \brief save simbody state for reconstructing simbody model graph
      private: std::vector<double> simbodyU;
    };
    /// \}
  }
}
#endif
