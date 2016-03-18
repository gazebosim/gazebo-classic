/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_SIMBODY_SIMBODYJOINT_HH_
#define _GAZEBO_PHYSICS_SIMBODY_SIMBODYJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class
    class SimbodyJointPrivate;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief Base class for all joints
    class GZ_PHYSICS_VISIBLE SimbodyJoint : public Joint
    {
      /// \brief Constructor
      public: SimbodyJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr JointLink(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      // Documentation inherited.
      public: virtual void Detach();

      // Documentation inherited.
      public: virtual void SetAnchor(const unsigned int _index,
                  const ignition::math::Vector3d &_anchor);

      // Documentation inherited.
      public: virtual void SetDamping(const unsigned int _index,
                                      const double _damping);

      // Documentation inherited.
      public: virtual void SetStiffness(const unsigned int _index,
                                        const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(const unsigned int _index,
        const double _stiffness, const double _damping,
        const double _reference = 0);

      // Documentation inherited.
      public: virtual ignition::math::Vector3d Anchor(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d LinkForce(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d LinkTorque(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                  const unsigned int _index,
                  const boost::any &_value);

      // Documentation inherited.
      public: virtual double Param(const std::string &_key,
                  const unsigned int _index) const;

      // Save current Simbody State
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      // Restore saved Simbody State
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      // Documentation inherited.
      public: virtual void SetForce(
                  const unsigned int _index, const double _force);

      // Documentation inherited.
      public: virtual double Force(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(const unsigned int _index,
                                   const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual JointWrench ForceTorque(const unsigned int _index) const;

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      /// internal force, e.g. damping forces.  This way, Joint::appliedForce
      /// keep track of external forces only.
      protected: virtual void SetForceImpl(const unsigned int _index,
                                           const double _force) = 0;

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(const unsigned int _index, const double _force);

      // Documentation inherited.
      public: virtual void CacheForceTorque();


      // Documentation inherited.
      public: virtual bool SetHighStop(const unsigned int _index,
                                       const ignition::math::Angle &_angle);

      // Documentation inherited.
      public: virtual bool SetLowStop(const unsigned int _index,
                                      const ignition::math::Angle &_angle);

      // Documentation inherited.
      public: virtual ignition::math::Angle HighStop(
                  const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Angle LowStop(
                  const unsigned int _index) const;

      /// \brief Set the physics initialized flag
      /// \param[in] _value True or false value for the
      /// physics initialized flag.
      public: void SetPhysicsInitialized(const bool _value);

      /// \internal
      /// \brief Private data pointer
      protected: SimbodyJointPrivate *simbodyJointDPtr;
    };
    /// \}
  }
}
#endif
