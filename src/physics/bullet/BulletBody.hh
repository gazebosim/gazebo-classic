/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Bullet Link class
 * Author: Nate Koenig
 * Date: 15 May 2009
 * SVN: $Id$
 */

#ifndef BulletBODY_HH
#define BulletBODY_HH

#include "BulletPhysics.hh"
#include "Link.hh"

namespace gazebo
{
  namespace physics
{
  class XMLConfigNode;
  class BulletMotionState;
  class BulletCollision;

  /// \addtogroup gazebo_physics
  /// \brief The body class
  /// \{
  /// Link class
  class BulletLink : public Link
  {
    /// \brief Constructor
    public: BulletLink(Entity *parent);

    /// \brief Destructor
    public: virtual ~BulletLink();

    /// \brief Load the body based on an common::XMLConfig node
    /// \param node common::XMLConfigNode pointer
    public: virtual void Load(common::XMLConfigNode *node);

    /// \brief Initialize the body
    public: virtual void Init();

    /// \brief Finalize the body
    public: virtual void Fini();

    /// \brief Update the body
    public: virtual void Update();

    /// \brief Attach a collision to this body
    /// \param collision Collisionetery to attach to this body
    public: virtual void AttachCollision(Collision *collision);

    /// \brief Called when the pose of the entity (or one of its parents) has
    /// changed
    public: virtual void OnPoseChange();

    /// \brief Set whether this body is enabled
    public: virtual void SetEnabled(bool enable) const;

    /// \brief Get whether this body is enabled in the physics engine
    public: virtual bool GetEnabled() const {return true;}
    /// \brief Update the center of mass
    public: virtual void UpdateCoM();

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

    /// \brief Get the bullet rigid body
    public: btRigidLink *GetBulletLink() const;

    /// \brief Set the linear damping factor
    public: virtual void SetLinearDamping(double damping);

    /// \brief Set the angular damping factor
    public: virtual void SetAngularDamping(double damping);

    /// \brief Set the relative pose of a child collision.
    public: void SetCollisionRelativePose(BulletCollision *collision,
                                          const math::Pose &newPose);

    private: btCompoundShape *compoundShape;
    private: BulletMotionState *motionState;
    private: btRigidLink *rigidLink;
    private: BulletPhysics *bulletPhysics;

    protected: math::Pose pose;
  };

  /// \}
}

}
}
#endif


