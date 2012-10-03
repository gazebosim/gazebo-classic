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
/* Desc: ODE Link class
 * Author: Nate Koenig
 */

#ifndef ODELINK_HH
#define ODELINK_HH

#include "physics/ode/ode_inc.h"
#include "physics/ode/ODETypes.hh"
#include "physics/Link.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \brief ODE physics engine wrapper
    /// \{

    /// \brief ODE Link class
    class ODELink : public Link
    {
      /// \brief Constructor
      public: ODELink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~ODELink();

      /// \brief Load the link based on SDF parameters
      /// \param _sdf the sdf parameters
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the link
      public: virtual void Init();

      /// \brief Finalize the link
      public: virtual void Fini();

      /// \brief Update the link
      public: virtual void Update();

      /// \brief Called when the pose of the entity (or one of its parents) has
      /// changed
      public: virtual void OnPoseChange();

      /// \brief Return the ID of this link
      /// \return ODE link id
      public: dBodyID GetODEId() const;

      /// \brief Set whether this link is enabled
      public: virtual void SetEnabled(bool enable) const;

      /// \brief Get whether this link is enabled in the physics engine
      public: virtual bool GetEnabled() const;

      /// \brief Update the mass matrix
      public: virtual void UpdateMass();

      /// \brief Update other parameters for ODE
      public: virtual void UpdateSurface();

      /// \brief Set the linear velocity of the link
      public: virtual void SetLinearVel(const math::Vector3 &vel);

      /// \brief Set the angular velocity of the link
      public: virtual void SetAngularVel(const math::Vector3 &vel);

      /// \brief Set the force applied to the link
      public: virtual void SetForce(const math::Vector3 &_force);

      /// \brief Set the torque applied to the link
      public: virtual void SetTorque(const math::Vector3 &_torque);

      /// \brief Add a force to the body
      public: virtual void AddForce(const math::Vector3 &_force);

      /// \brief Add a force to the body
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      /// \brief Add a force to the body using a global position
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      /// \brief Set the force applied to the body (add by Stefano)
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force, const math::Vector3 &_relpos);

      /// \brief Add a torque to the body
      public: virtual void AddTorque(const math::Vector3 &_torque);

      /// \brief Add a torque to the body relative frame
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      /// \brief Get the linear velocity of the link in the world frame
      public: virtual math::Vector3 GetWorldLinearVel() const;

      /// \brief Get the angular velocity of the link in the world frame
      public: virtual math::Vector3 GetWorldAngularVel() const;

      /// \brief Get the force applied to the link in the world frame
      public: virtual math::Vector3 GetWorldForce() const;

      /// \brief Get the torque applied to the link in the world frame
      public: virtual math::Vector3 GetWorldTorque() const;

      /// \brief Set whether gravity affects this link
      public: virtual void SetGravityMode(bool mode);

      /// \brief Get the gravity mode
      public: virtual bool GetGravityMode();

      /// \brief Set whether this link will collide with others in the model
      public: void SetSelfCollide(bool collide);

      /// \brief Get the link's space ID
      public: dSpaceID GetSpaceId() const;

      /// \brief Set the link's space ID
      public: void SetSpaceId(dSpaceID spaceid);

      /// \brief Set the linear damping factor
      public: virtual void SetLinearDamping(double damping);

      /// \brief Set the angular damping factor
      public: virtual void SetAngularDamping(double damping);

      /// \brief callback when ODE determines a body is disabled
      public: static void DisabledCallback(dBodyID _id);

      /// \brief when ODE updates dynamics bodies, this callback
      ///        propagates the chagnes in pose back to Gazebo
      public: static void MoveCallback(dBodyID id);

      /// \brief Set whether this link is in the kinematic state
      public: virtual void SetKinematic(const bool &state);

      /// \brief Get whether this link is in the kinematic state
      public: virtual bool GetKinematic() const;

      /// \brief Allow the link to auto disable.
      /// \param _disable If true, the link is allowed to auto disable.
      public: virtual void SetAutoDisable(bool _disable);

      protected: math::Pose pose;

      /// ODE link handle
      private: dBodyID linkId;

      private: ODEPhysicsPtr odePhysics;

      private: dSpaceID spaceId;
    };

    /// \}
  }
}
#endif
