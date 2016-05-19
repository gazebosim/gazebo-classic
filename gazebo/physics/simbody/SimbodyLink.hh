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
#ifndef _GAZEBO_PHYSICS_SIMBODY_SIMBODYLINK_HH_
#define _GAZEBO_PHYSICS_SIMBODY_SIMBODYLINK_HH_

#include <vector>

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class
    class SimbodyLinkPrivate;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \brief simbody physics engine wrapper
    /// \{

    /// \brief Simbody Link class
    class GZ_PHYSICS_VISIBLE SimbodyLink : public Link
    {
      /// \brief Constructor
      public: explicit SimbodyLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyLink();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetEnabled(const bool enable) const;

      // Documentation inherited.
      public: virtual bool Enabled() const;

      // Documentation inherited.
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void SetTorque(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldLinearVel(
        const ignition::math::Vector3d &_vector3) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset,
          const ignition::math::Quaterniond &_q) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldCoGLinearVel() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldForce() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldTorque() const;

      // Documentation inherited.
      public: virtual void SetGravityMode(const bool _mode);

      // Documentation inherited.
      public: virtual bool GravityMode() const;

      // Documentation inherited.
      public: virtual void SetSelfCollide(const bool _collide);

      // Documentation inherited.
      public: virtual void SetLinearDamping(const double _damping);

      // Documentation inherited.
      public: virtual void SetAngularDamping(const double _damping);

      // Documentation inherited.
      public: virtual void AddForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddRelativeForce(
                  const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddForceAtWorldPosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_pos);

      // Documentation inherited.
      public: virtual void AddForceAtRelativePosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_offset =
                  ignition::math::Vector3d::Zero);

      // Documentation inherited.
      public: virtual void AddTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void AddRelativeTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void SetAutoDisable(const bool _disable);

      // Documentation inherited.
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      // Documentation inherited.
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      /// \brief If the inboard body of this link is ground, simply
      /// lock the inboard joint to freeze it to ground.  Otherwise,
      /// add a weld constraint to simulate freeze to ground effect.
      /// \param[in] _static if true, freeze link to ground.  Otherwise
      /// unfreeze link.
      public: virtual void SetLinkStatic(const bool _static);

      /// \brief Convert Gazebo Inertia to Simbody MassProperties
      /// Where Simbody MassProperties contains mass,
      /// center of mass location, and unit inertia about body origin.
      public: SimTK::MassProperties MassProperties() const;

      public: SimTK::MassProperties EffectiveMassProps(
                  const int _numFragments) const;

      public: void SetDirtyPose(const ignition::math::Pose3d &_pose);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Set the physics initialized flag
      /// \param[in] _value True or false value for the
      /// physics initialized flag.
      public: void SetPhysicsInitialized(const bool _value);

      /// \brief Get reference to mobilized body
      /// \return Reference to mobilized body
      public: SimTK::MobilizedBody &MobilizedBody() const;

      /// \brief Get the must be base link flag.
      /// \return True if the link must be a base link.
      public: bool MustBeBaseLink() const;

      /// \brief Get the number of slave mobilizers
      /// \retur Number of slave mobilizers
      public: size_t SlaveMobodsCount() const;

      /// \brief Add a new slave mobilizer
      /// \param[in] _mobod New slave mobilizer
      public: void AddSlaveMobod(SimTK::MobilizedBody _mobod);

      /// \brief Get a slave mobilizer.
      /// \param[in] _index Index of the slave mobilizer
      /// \return The slave mobilizer
      public: SimTK::MobilizedBody &SlaveMobod(unsigned int _index) const;

      /// \brief Add a new slave weld
      /// \param[in] _weld New slave weld
      public: void AddSlaveWeld(SimTK::Constraint::Weld _weld);

      /// \brief Internal call to change effect of gravity on Link
      /// based on gravityMode if gravityModeDirty is true.
      private: void ProcessSetGravityMode();

      /// \brief Internal call to set link static
      /// based on staticLink if staticLinkDirty is true.
      private: void ProcessSetLinkStatic();

      /// \internal
      /// \brief Private data pointer
      private: SimbodyLinkPrivate *simbodyLinkDPtr;
    };
    /// \}
  }
}
#endif
