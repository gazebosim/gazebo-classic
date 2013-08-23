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
/* Desc: Simbody Link class
 * Author: Nate Koenig
 * Date: 15 May 2009
 */

#ifndef _SIMBODYLINK_HH_
#define _SIMBODYLINK_HH_

#include <Simbody.h>

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/Link.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \brief simbody physics engine wrapper
    /// \{

    /// \brief Simbody Link class
    class SimbodyLink : public Link
    {
      /// \brief Constructor
      public: SimbodyLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyLink();

      /// \brief Load the body based on an common::XMLConfig node
      public: virtual void Load(sdf::ElementPtr _ptr);

      /// \brief Initialize the body
      public: virtual void Init();

      /// \brief Finalize the body
      public: virtual void Fini();

      /// \brief Called when the pose of the entity (or one of its parents) has
      /// changed
      public: virtual void OnPoseChange();

      /// \brief Set whether this body is enabled
      public: virtual void SetEnabled(bool enable) const;

      /// \brief Get whether this body is enabled in the physics engine
      public: virtual bool GetEnabled() const {return true;}

      /// \brief Set the linear velocity of the body
      public: virtual void SetLinearVel(const math::Vector3 &vel);

      /// \brief Set the angular velocity of the body
      public: virtual void SetAngularVel(const math::Vector3 &vel);

      /// \brief Set the force applied to the body
      public: virtual void SetForce(const math::Vector3 &force);

      /// \brief Set the torque applied to the body
      public: virtual void SetTorque(const math::Vector3 &force);

      /// \brief Get the linear velocity of the body in the world frame
      public: virtual math::Vector3 GetWorldLinearVel(
        const math::Vector3& _vector3) const;

      /// \brief Get the linear velocity of the body in the world frame
      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset,
          const math::Quaternion &_q) const;

      /// \brief Get the linear velocity of the body in the world frame
      public: virtual math::Vector3 GetWorldCoGLinearVel() const;

      /// \brief Get the angular velocity of the body in the world frame
      public: virtual math::Vector3 GetWorldAngularVel() const;

      /// \brief Get the force applied to the body in the world frame
      public: virtual math::Vector3 GetWorldForce() const;

      /// \brief Get the torque applied to the body in the world frame
      public: virtual math::Vector3 GetWorldTorque() const;

      /// \brief Set whether gravity affects this body
      public: virtual void SetGravityMode(bool mode);

      /// \brief Get the gravity mode
      public: virtual bool GetGravityMode() const;

      /// \brief store gravity mode given link might not be around
      public: bool gravityMode;

      /// \brief Set whether this body will collide with others in the model
      public: void SetSelfCollide(bool collide);

      /// \brief Set the linear damping factor
      public: virtual void SetLinearDamping(double damping);

      /// \brief Set the angular damping factor
      public: virtual void SetAngularDamping(double damping);

      /// \brief Set the relative pose of a child collision.
      /*public: void SetCollisionRelativePose(SimbodyCollision *collision,
                                            const math::Pose &newPose);
                                            */

      /// \brief Add a force to the body
      public: virtual void AddForce(const math::Vector3 &_force);

      /// \brief Add a force to the body, components are relative to the
      ///        body's own frame of reference.
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      /// \brief Add a force to the body using a global position
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      /// \brief Add a force to the body at position expressed to the body's
      ///        own frame of reference.
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relpos);

      /// \brief Add a torque to the body
      public: virtual void AddTorque(const math::Vector3 &_torque);

      /// \brief Add a torque to the body, components are relative to the
      ///        body's own frame of reference.
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      /// \copydoc Link::SetAutoDisable(bool)
      public: virtual void SetAutoDisable(bool _disable);

      /// \brief keep a pointer to the simbody physics engine for convenience
      private: SimbodyPhysicsPtr simbodyPhysics;

      /// \brief: Force this link to be a base body, where its inboard
      /// body is the world with 6DOF.
      public: bool mustBeBaseLink;

      // Below to be filled in after everything is loaded
      // Which MobilizedBody corresponds to the master instance of this link.
      public: SimTK::MobilizedBody                   masterMobod;

      // Keeps track if physics has been initialized
      public: bool physicsInitialized;

      // If this link got split into a master and slaves, these are the 
      // MobilizedBodies used to mobilize the slaves.
      public: std::vector<SimTK::MobilizedBody>      slaveMobods;

      // And these are the Weld constraints used to attach slaves to master.
      public: std::vector<SimTK::Constraint::Weld>   slaveWelds;

      // Convert Gazebo Inertia to Simbody MassProperties
      // Where Simbody MassProperties contains mass,
      // center of mass location, and unit inertia about body origin.
      public: SimTK::MassProperties GetMassProperties() const;
      public: SimTK::MassProperties GetEffectiveMassProps(int _numFragments) const;
      public: void SetDirtyPose(math::Pose _pose)
              {
                this->dirtyPose = _pose;
              }
    };
    /// \}
  }
}
#endif
