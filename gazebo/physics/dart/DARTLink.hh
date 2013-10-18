/*
 * Copyright 2012-2013 Open Source Robotics Foundation
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

#ifndef _DARTLINK_HH_
#define _DARTLINK_HH_

#include "gazebo/physics/Link.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_dart DART Physics
    /// \brief dart physics engine wrapper
    /// \{

    /// \brief DART Link class
    class DARTLink : public Link
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
      public: virtual void SetLinearVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void SetTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
          const math::Vector3 &_force,
          const math::Vector3 &_relpos);

      // Documentation inherited
      public: virtual void AddTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(const math::Vector3& _torque);

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3& _offset = math::Vector3(0, 0, 0)) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset,
          const math::Quaternion &_q) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldCoGLinearVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldTorque() const;

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

      // TODO: (in test)
      public: void updateDirtyPoseFromDARTTransformation();

      /// \brief
      public: dart::dynamics::BodyNode* GetBodyNode() const;

      /// \brief
      public: DARTPhysicsPtr GetDARTPhysics(void) const;

      /// \brief
      public: dart::simulation::World* GetDARTWorld(void) const;

      /// \brief
      public: DARTModelPtr GetDARTModel() const;

      /// \brief
      public: dart::dynamics::BodyNode* GetDARTBodyNode() const;

      /// \brief
      public: void SetDARTParentJoint(DARTJointPtr _dartParentJoint);

      /// \brief
      public: void AddDARTChildJoint(DARTJointPtr _dartChildJoint);

      /// \brief Pointer to the DART physics engine.
      private: DARTPhysicsPtr dartPhysics;

      /// \brief
      private: dart::dynamics::BodyNode* dtBodyNode;

      /// \brief
      private: DARTJointPtr dartParentJoint;

      /// \brief
      private: std::vector<DARTJointPtr> dartChildJoints;

      /// \brief Weld joint for static link mode.
      private: dart::dynamics::WeldJoint* dtStaticJoint;

      /// \brief
      private: dart::dynamics::Joint* dtDynamicJoint;

      /// \biref If true, freeze link to world (inertial) frame.
      private: bool staticLink;
    };
    /// \}
  }
}
#endif
