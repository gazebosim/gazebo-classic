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
      public: explicit SimbodyJoint(BasePtr _parent);

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

      /// \brief Get the mustBreakLoopHere flag.
      /// \return True if a loop must be broken at this joint.
      public: bool MustBreakLoopHere() const;

      /// \brief Get the parent body frame to mobilizer frame transform.
      /// \return Parent body frame to mobilizer frame transform.
      public: SimTK::Transform XPA() const;

      /// \brief Get the child body frame to mobilizer frame transform.
      /// \return Child body frame to mobilizer frame transform.
      public: SimTK::Transform XCB() const;

      /// \brief Get the default mobilizer pose.
      /// \return Default mobilizer pose.
      public: SimTK::Transform DefxAB() const;

      /// \brief Get the limit force for an axis.
      /// \param[in] _index Index of the axis
      /// \return Limit force for the specified joint axis.
      public: SimTK::Force::MobilityLinearStop LimitForce(
                  const unsigned int _index) const;

      /// \brief Set the limit force for an axis.
      /// \param[in] _index Index of the axis
      /// \param[in] _limit New limit forces value
      public: void SetLimitForce(const unsigned int _index,
                  const SimTK::Force::MobilityLinearStop _limit);

      /// \brief Get the damper value for an axis.
      /// \param[in] _index Index of the axis
      /// \return Damper value for the specified joint axis.
      public: SimTK::Force::MobilityLinearDamper Damper(
                  const unsigned int _index) const;

      /// \brief Set the damper value for an axis.
      /// \param[in] _index Index of the axis
      /// \param[in] _damper New damper value
      public: void SetDamper(const unsigned int _index,
                  const SimTK::Force::MobilityLinearDamper _damper);

      /// \brief Get the spring value for an axis.
      /// \param[in] _index Index of the axis
      /// \return Spring value for the specified joint axis.
      public: SimTK::Force::MobilityLinearSpring Spring(
                  const unsigned int _index) const;

      /// \brief Set the spring value for an axis.
      /// \param[in] _index Index of the axis
      /// \param[in] _spring New spring value
      public: void SetSpring(const unsigned int _index,
                  const SimTK::Force::MobilityLinearSpring  _spring);

      /// \brief Set the mobilizer.
      /// \param[in] _mobod New mobilizer
      public: void SetMobod(const SimTK::MobilizedBody &_mobod);

      /// \brief Return whether the parent & child relationship is reversed.
      /// \return True if parent & child relationship is reversed.
      public: bool IsReversed() const;

      /// \brief Set whether the parent & child relationship is reversed.
      /// \param[in] _value True if parent & child relationship is reversed.
      public: void SetIsReversed(const bool _value);

      /// \internal
      /// \brief Private data pointer
      protected: SimbodyJointPrivate *simbodyJointDPtr;
    };
    /// \}
  }
}
#endif
