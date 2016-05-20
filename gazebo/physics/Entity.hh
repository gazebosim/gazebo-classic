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
/* Desc: Base class for all physical entities
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#ifndef _ENTITY_HH_
#define _ENTITY_HH_

#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>

#include <boost/function.hpp>
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/UpdateInfo.hh"

#include "gazebo/math/MathTypes.hh"
#include "gazebo/math/Box.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Base.hh"
#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Entity Entity.hh physics/physics.hh
    /// \brief Base class for all physics objects in Gazebo.
    class GZ_PHYSICS_VISIBLE Entity : public Base
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the entity.
      public: explicit Entity(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~Entity();

      /// \brief Load the entity.
      /// \param[in] _sdf Pointer to an SDF element.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Finalize the entity.
      public: virtual void Fini();

      /// \brief Reset the entity.
      public: virtual void Reset();
      using Base::Reset;

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the name of the entity.
      /// \param[in] _name The new name.
      public: virtual void SetName(const std::string &_name);

      /// \brief Set whether this entity is static: immovable.
      /// \param[in] _static True = static.
      public: void SetStatic(const bool &_static);

      /// \brief Return whether this entity is static.
      /// \return True if static.
      public: bool IsStatic() const;

      /// \brief Set the initial pose.
      /// \param[in] _pose The initial pose.
      public: void SetInitialRelativePose(const math::Pose &_pose);

      /// \brief Get the initial relative pose.
      /// \return The initial relative pose.
      public: math::Pose GetInitialRelativePose() const;

      /// \brief Return the bounding box for the entity.
      /// \return The bounding box.
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get the absolute pose of the entity.
      /// \return The absolute pose of the entity.
      public: inline virtual const math::Pose &GetWorldPose() const
              {return this->worldPose;}

      /// \brief Get the pose of the entity relative to its parent.
      /// \return The pose of the entity relative to its parent.
      public: math::Pose GetRelativePose() const;

      /// \brief Set the pose of the entity relative to its parent.
      /// \param[in] _pose The new pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetRelativePose(const math::Pose &_pose,
                                   bool _notify = true,
                                   bool _publish = true);

      /// \brief Set the world pose of the entity.
      /// \param[in] _pose The new world pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetWorldPose(const math::Pose &_pose,
                                bool _notify = true,
                                bool _publish = true);

      /// \brief Get the linear velocity of the entity.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual math::Vector3 GetRelativeLinearVel() const
              {return math::Vector3();}

      /// \brief Get the linear velocity of the entity in the world frame.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual math::Vector3 GetWorldLinearVel() const
              {return math::Vector3();}

      /// \brief Get the angular velocity of the entity.
      /// \return A math::Vector3 for the velocity.
      public: virtual math::Vector3 GetRelativeAngularVel() const
              {return math::Vector3();}

      /// \brief Get the angular velocity of the entity in the world frame.
      /// \return A math::Vector3 for the velocity.
      public: virtual math::Vector3 GetWorldAngularVel() const
              {return math::Vector3();}

      /// \brief Get the linear acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              {return math::Vector3();}

      /// \brief Get the linear acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetWorldLinearAccel() const
              {return math::Vector3();}

      /// \brief Get the angular acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              {return math::Vector3();}

      /// \brief Get the angular acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual math::Vector3 GetWorldAngularAccel() const
              {return math::Vector3();}

      /// \brief Set to true if this entity is a canonical link for a model.
      /// \param[in] _value True if the link is canonical.
      public: void SetCanonicalLink(bool _value);

      /// \brief A helper function that checks if this is a canonical body.
      /// \return True if the link is canonical.
      public: inline bool IsCanonicalLink() const
              {return this->isCanonicalLink;}

      /// \brief Set an animation for this entity.
      /// \param[in] _anim Pose animation.
      /// \param[in] _onComplete Callback for when the animation completes.
      public: void SetAnimation(const common::PoseAnimationPtr &_anim,
                                boost::function<void()> _onComplete);

      /// \brief Set an animation for this entity.
      /// \param[in] _anim Pose animation.
      public: void SetAnimation(common::PoseAnimationPtr _anim);

      /// \brief Stop the current animation, if any.
      public: virtual void StopAnimation();

      /// \brief Get the parent model, if one exists.
      /// \return Pointer to a model, or NULL if no parent model exists.
      public: ModelPtr GetParentModel();

      /// \brief Get a child collision entity, if one exists.
      /// \param[in] _name Name of the child collision object.
      /// \return Pointer to the Collision object, or NULL if not found.
      public: CollisionPtr GetChildCollision(const std::string &_name);

      /// \brief Get a child linke entity, if one exists.
      /// \param[in] _name Name of the child Link object.
      /// \return Pointer to the Link object, or NULL if not found.
      public: LinkPtr GetChildLink(const std::string &_name);

      /// \brief Get the distance to the nearest entity below
      /// (along the Z-axis) this entity.
      /// \param[out] _distBelow The distance to the nearest entity below.
      /// \param[out] _entityName The name of the nearest entity below.
      public: void GetNearestEntityBelow(double &_distBelow,
                                         std::string &_entityName);

      /// \brief Move this entity to be ontop of the nearest entity below.
      public: void PlaceOnNearestEntityBelow();

      /// \brief Move this entity to be ontop of another entity by name.
      /// \param[in] _entityName Name of the Entity this Entity should be
      /// ontop of.
      public: void PlaceOnEntity(const std::string &_entityName);

      /// \brief Returns collision bounding box.
      /// \return Collsiion boundin box.
      public: math::Box GetCollisionBoundingBox() const;

      /// \brief Set angular and linear rates of an physics::Entity.
      /// \param[in] _linear Linear twist.
      /// \param[in] _angular Angular twist.
      /// \param[in] _updateChildren True to pass this update to child
      /// entities.
      public: void SetWorldTwist(const math::Vector3 &_linear,
                                 const math::Vector3 &_angular,
                                 bool _updateChildren = true);

      /// \brief Returns Entity#dirtyPose.
      ///
      /// The dirty pose is the pose set by the physics engine before it's
      /// value is propagated to the rest of the simulator.
      /// \return The dirty pose of the entity.
      public: const math::Pose &GetDirtyPose() const;

      /// \brief This function is called when the entity's
      /// (or one of its parents) pose of the parent has changed.
      protected: virtual void OnPoseChange() = 0;

      /// \brief Publish the pose.
      private: virtual void PublishPose();

      /// \brief Helper function to get the collision bounding box.
      /// \param[in] _base Object to calculated the bounding box for.
      /// \return The boundin box for the passed in object.
      private: math::Box GetCollisionBoundingBoxHelper(BasePtr _base) const;

      /// \brief Set the world pose for a model.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseModel(const math::Pose &_pose,
                                      bool _notify,
                                      bool _publish);

      /// \brief Set the world pose for a canonical Link.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseCanonicalLink(const math::Pose &_pose,
                                              bool _notify, bool _publish);

      /// \brief Set the world pose for a common entity.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseDefault(const math::Pose &_pose, bool _notify,
                                        bool _publish);

      /// \brief Called when a new pose message arrives.
      /// \param[in] _msg The message to set the pose from.
      private: void OnPoseMsg(ConstPosePtr &_msg);

      /// \brief Handle a change of pose
      /// \param[in] update_children if set to true, will call OnPoseChange
      ///            for all children (1 level, non-recursive).
      ///            But if the Object is static, we force children
      ///            OnPoseChange call
      private: void UpdatePhysicsPose(bool update_children = true);

      /// \brief Update an animation.
      /// \param[in] _info Update information.
      private: void UpdateAnimation(const common::UpdateInfo &_info);

      /// \brief A helper that prevents numerous dynamic_casts.
      protected: EntityPtr parentEntity;

      /// \brief World pose of the entity.
      protected: mutable math::Pose worldPose;

      /// \brief Communication node.
      protected: transport::NodePtr node;

      /// \brief Visual publisher.
      protected: transport::PublisherPtr visPub;

      /// \brief Request publisher.
      protected: transport::PublisherPtr requestPub;

      /// \brief Visual message container.
      protected: msgs::Visual *visualMsg;

      /// \brief Current pose animation
      protected: common::PoseAnimationPtr animation;

      /// \brief Previous time an animation was updated.
      protected: common::Time prevAnimationTime;

      /// \brief Start pose of an animation.
      protected: math::Pose animationStartPose;

      /// \brief All our event connections.
      protected: std::vector<event::ConnectionPtr> connections;

      /// \brief Connection used to update an animation.
      protected: event::ConnectionPtr animationConnection;

      /// \brief The pose set by a physics engine.
      protected: math::Pose dirtyPose;

      /// \brief Scale of the entity
      protected: ignition::math::Vector3d scale;

      /// \brief True if the object is static.
      private: bool isStatic;

      /// \brief Only used by Links. Included here for performance.
      private: bool isCanonicalLink;

      /// \brief The initial pose of the entity.
      private: math::Pose initialRelativePose;

      /// \brief Pose publisher.
      private: transport::PublisherPtr posePub;

      /// \brief Pose subscriber.
      private: transport::SubscriberPtr poseSub;

      /// \brief Callback for when an animation completes.
      private: boost::function<void()> onAnimationComplete;

      /// \brief The function used to to set the world pose.
      private: void (Entity::*setWorldPoseFunc)(const math::Pose &, bool, bool);
    };
    /// \}
  }
}
#endif
