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

#include "physics/simbody/simbody_inc.h"
#include "physics/simbody/SimbodyTypes.hh"
#include "physics/Link.hh"

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

      /// \brief Update the body
      public: virtual void Update();

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
      public: virtual math::Vector3 GetWorldLinearVel() const;

      /// \brief Get the angular velocity of the body in the world frame
      public: virtual math::Vector3 GetWorldAngularVel() const;

      /// \brief Get the force applied to the body in the world frame
      public: virtual math::Vector3 GetWorldForce() const;

      /// \brief Get the torque applied to the body in the world frame
      public: virtual math::Vector3 GetWorldTorque() const;

      /// \brief Set whether gravity affects this body
      public: virtual void SetGravityMode(bool mode);

      /// \brief Get the gravity mode
      public: virtual bool GetGravityMode();

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

      private: SimbodyPhysicsPtr simbodyPhysics;
      protected: math::Pose pose;
    };
    /// \}
  }
}
#endif
