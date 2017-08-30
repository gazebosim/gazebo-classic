/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_DART_DARTLINK_HH_
#define GAZEBO_PHYSICS_DART_DARTLINK_HH_

#include <vector>

#include "gazebo/physics/Link.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTLinkPrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief DART Link class
    class GZ_PHYSICS_VISIBLE DARTLink : public Link
    {
      /// \brief Constructor
      public: explicit DARTLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~DARTLink();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited
      public: virtual bool GetEnabled() const;

      // Documentation inherited
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited
      public: virtual void SetForce(const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void SetTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual void AddForce(const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(
                  const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
          const ignition::math::Vector3d &_force,
          const ignition::math::Vector3d &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_offset =
                  ignition::math::Vector3d::Zero);

      // Documentation inherited
      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset =
          ignition::math::Vector3d::Zero) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset,
          const ignition::math::Quaterniond &_q) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldCoGLinearVel() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldForce() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldTorque() const;

      // Documentation inherited
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited
      public: virtual bool GetGravityMode() const;

      // Documentation inherited
      public: virtual void SetSelfCollide(bool _collide);

      // Documentation inherited
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited
      public: virtual void SetKinematic(const bool &_state);

      // Documentation inherited
      public: virtual bool GetKinematic() const;

      // Documentation inherited
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Store DART Transformation to Entity::dirtyPose and add this
      ///        link to World::dirtyPoses so that World::Update() trigger
      ///        Entity::SetWorldPose() for this link.
      public: void updateDirtyPoseFromDARTTransformation();

      /// \brief Get pointer to DART Physics engine associated with this link.
      /// \return Pointer to the DART Physics engine.
      public: DARTPhysicsPtr GetDARTPhysics(void) const;

      /// \brief Get pointer to DART World associated with this link.
      /// \return Pointer to the DART World.
      public: dart::simulation::WorldPtr DARTWorld(void) const;

      /// \brief Get pointer to DART Model associated with this link.
      /// \return Pointer to the DART Model.
      public: DARTModelPtr GetDARTModel() const;

      /// \brief Get DART BodyNode properties
      public: DARTBodyNodePropPtr DARTProperties() const;

      /// \brief Set pointer to DART BodyNode associated with this link.
      /// \param[in] Pointer to DART BodyNode.
      public: void SetDARTBodyNode(dart::dynamics::BodyNode *_dtBodyNode);

      /// \brief Add pointer to a BodyNode representing a fragment of this link.
      /// \param[in] Pointer to DART BodyNode.
      public: void AddSlaveBodyNode(dart::dynamics::BodyNode *_dtBodyNode);

      /// \brief Get pointer to DART BodyNode associated with this link.
      /// \return Pointer to DART BodyNode.
      public: dart::dynamics::BodyNode *DARTBodyNode() const;

      /// \brief Set parent joint of this link.
      /// \param[in] _dartParentJoint Pointer to the parent joint.
      public: void SetDARTParentJoint(DARTJointPtr _dartParentJoint);

      /// \brief Set child joint of this link.
      /// \param[in] _dartChildJoint Pointer to the child joint.
      public: void AddDARTChildJoint(DARTJointPtr _dartChildJoint);

      /// \brief Get whether this link is soft body.
      /// \brief True if this link is soft body.
      public: bool IsSoftBody() const;

      /// \internal
      /// \brief Pointer to private data
      private: DARTLinkPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
