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
#ifndef _GAZEBO_PHYSICS_ENTITY_HH_
#define _GAZEBO_PHYSICS_ENTITY_HH_

#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>

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

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class.
    class EntityPrivate;

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
      /// \deprecated See version that accepts ignition::math parameters.
      public: void SetInitialRelativePose(const math::Pose &_pose)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Set the initial pose.
      /// \param[in] _pose The initial pose.
      public: void SetInitialRelativePose(const ignition::math::Pose3d &_pose);

      /// \brief Get the initial relative pose.
      /// \return The initial relative pose.
      /// \deprecated See InitialRelativePose()
      public: math::Pose GetInitialRelativePose() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the initial relative pose.
      /// \return The initial relative pose.
      public: ignition::math::Pose3d InitialRelativePose() const;

      /// \brief Return the bounding box for the entity.
      /// \return The bounding box.
      /// \deprecated See BoundingBox()
      public: virtual math::Box GetBoundingBox() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return the bounding box for the entity.
      /// \return The bounding box.
      public: virtual ignition::math::Box BoundingBox() const;

      /// \brief Get the absolute pose of the entity.
      /// \return The absolute pose of the entity.
      /// \deprecated See WorldPose();
      public: virtual math::Pose GetWorldPose() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the absolute pose of the entity.
      /// \return The absolute pose of the entity.
      public: virtual const ignition::math::Pose3d &WorldPose() const;

      /// \brief Get the pose of the entity relative to its parent.
      /// \return The pose of the entity relative to its parent.
      /// \deprecated See RelativePose()
      public: math::Pose GetRelativePose() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the pose of the entity relative to its parent.
      /// \return The pose of the entity relative to its parent.
      public: ignition::math::Pose3d RelativePose() const;

      /// \brief Set the pose of the entity relative to its parent.
      /// \param[in] _pose The new pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      /// \deprecated See version that accepts ignition::math parameters.
      public: void SetRelativePose(const math::Pose &_pose,
                                   bool _notify = true,
                                   bool _publish = true) GAZEBO_DEPRECATED(7.0);

      /// \brief Set the pose of the entity relative to its parent.
      /// \param[in] _pose The new pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetRelativePose(const ignition::math::Pose3d &_pose,
                                   const bool _notify = true,
                                   const bool _publish = true);

      /// \brief Set the world pose of the entity.
      /// \param[in] _pose The new world pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      /// \deprecated See version that accepts ignition::math parameters
      public: void SetWorldPose(const math::Pose &_pose,
                                bool _notify = true,
                                bool _publish = true) GAZEBO_DEPRECATED(7.0);

      /// \brief Set the world pose of the entity.
      /// \param[in] _pose The new world pose.
      /// \param[in] _notify True = tell children of the pose change.
      /// \param[in] _publish True to publish the pose.
      public: void SetWorldPose(const ignition::math::Pose3d &_pose,
                                const bool _notify = true,
                                const bool _publish = true);

      /// \brief Get the linear velocity of the entity.
      /// \return A math::Vector3 for the linear velocity.
      /// \deprecated See RelativeLinearVel()
      public: virtual math::Vector3 GetRelativeLinearVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear velocity of the entity.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual ignition::math::Vector3d RelativeLinearVel() const;

      /// \brief Get the linear velocity of the entity in the world frame.
      /// \return A math::Vector3 for the linear velocity.
      /// \deprecated See WorldLinearVel()
      public: virtual math::Vector3 GetWorldLinearVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear velocity of the entity in the world frame.
      /// \return A math::Vector3 for the linear velocity.
      public: virtual ignition::math::Vector3d WorldLinearVel() const;

      /// \brief Get the angular velocity of the entity.
      /// \return A math::Vector3 for the velocity.
      /// \deprecated See RelativeAngularVel()
      public: virtual math::Vector3 GetRelativeAngularVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular velocity of the entity.
      /// \return A math::Vector3 for the velocity.
      public: virtual ignition::math::Vector3d RelativeAngularVel() const;

      /// \brief Get the angular velocity of the entity in the world frame.
      /// \return A math::Vector3 for the velocity.
      /// \deprecated See WorldAngularVel()
      public: virtual math::Vector3 GetWorldAngularVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular velocity of the entity in the world frame.
      /// \return A math::Vector3 for the velocity.
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      /// \brief Get the linear acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      /// \deprecated See RelativeLinearAccel()
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual ignition::math::Vector3d RelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      /// \deprecated See WorldLinearAccel();
      public: virtual math::Vector3 GetWorldLinearAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual ignition::math::Vector3d WorldLinearAccel() const;

      /// \brief Get the angular acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      /// \deprecated See RelativeAngularAccel()
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular acceleration of the entity.
      /// \return A math::Vector3 for the acceleration.
      public: virtual ignition::math::Vector3d RelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      /// \deprecated See WorldAngularAccel()
      public: virtual math::Vector3 GetWorldAngularAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular acceleration of the entity in the world frame.
      /// \return A math::Vector3 for the acceleration.
      public: virtual ignition::math::Vector3d WorldAngularAccel() const;

      /// \brief Set to true if this entity is a canonical link for a model.
      /// \param[in] _value True if the link is canonical.
      public: void SetCanonicalLink(const bool _value);

      /// \brief A helper function that checks if this is a canonical body.
      /// \return True if the link is canonical.
      public: bool IsCanonicalLink() const;

      /// \brief Set an animation for this entity.
      /// \param[in] _anim Pose animation.
      /// \param[in] _onComplete Callback for when the animation completes.
      public: void SetAnimation(const common::PoseAnimationPtr &_anim,
                                std::function<void()> _onComplete);

      /// \brief Set an animation for this entity.
      /// \param[in] _anim Pose animation.
      public: void SetAnimation(common::PoseAnimationPtr _anim);

      /// \brief Stop the current animation, if any.
      public: virtual void StopAnimation();

      /// \brief Get the parent model, if one exists.
      /// \return Pointer to a model, or NULL if no parent model exists.
      /// \deprecated See ParentModel()
      public: ModelPtr GetParentModel() GAZEBO_DEPRECATED(7.0);

      /// \brief Get the parent model, if one exists.
      /// \return Pointer to a model, or NULL if no parent model exists.
      public: Model *ParentModel();

      /// \brief Get a child collision entity, if one exists.
      /// \param[in] _name Name of the child collision object.
      /// \return Pointer to the Collision object, or NULL if not found.
      /// \deprecated See ChildCollision(const std::string)
      public: CollisionPtr GetChildCollision(const std::string &_name)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get a child collision entity, if one exists.
      /// \param[in] _name Name of the child collision object.
      /// \return Pointer to the Collision object, or NULL if not found.
      public: CollisionPtr ChildCollision(const std::string &_name) const;

      /// \brief Get a child linke entity, if one exists.
      /// \param[in] _name Name of the child Link object.
      /// \return Pointer to the Link object, or NULL if not found.
      /// \deprecated See ChildLink(const std::string)
      public: LinkPtr GetChildLink(const std::string &_name)
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get a child linke entity, if one exists.
      /// \param[in] _name Name of the child Link object.
      /// \return Pointer to the Link object, or NULL if not found.
      public: LinkPtr ChildLink(const std::string &_name) const;

      /// \brief Get the distance to the nearest entity below
      /// (along the Z-axis) this entity.
      /// \param[out] _distBelow The distance to the nearest entity below.
      /// \param[out] _entityName The name of the nearest entity below.
      /// \deprecated See NearestEntityBelow(double &, std::string &)
      public: void GetNearestEntityBelow(double &_distBelow,
                  std::string &_entityName) GAZEBO_DEPRECATED(7.0);

      /// \brief Get the distance to the nearest entity below
      /// (along the Z-axis) this entity.
      /// \param[out] _distBelow The distance to the nearest entity below.
      /// \param[out] _entityName The name of the nearest entity below.
      public: void NearestEntityBelow(double &_distBelow,
                  std::string &_entityName) const;

      /// \brief Move this entity to be ontop of the nearest entity below.
      public: void PlaceOnNearestEntityBelow();

      /// \brief Move this entity to be ontop of another entity by name.
      /// \param[in] _entityName Name of the Entity this Entity should be
      /// ontop of.
      public: void PlaceOnEntity(const std::string &_entityName);

      /// \brief Returns collision bounding box.
      /// \return Collsiion boundin box.
      /// \deprecated See CollisionBoundingBox()
      public: math::Box GetCollisionBoundingBox() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns collision bounding box.
      /// \return Collsiion boundin box.
      public: ignition::math::Box CollisionBoundingBox() const;

      /// \brief Set angular and linear rates of an physics::Entity.
      /// \param[in] _linear Linear twist.
      /// \param[in] _angular Angular twist.
      /// \param[in] _updateChildren True to pass this update to child
      /// entities.
      /// \deprecated See version that accepts ignition::math parameters
      public: void SetWorldTwist(const math::Vector3 &_linear,
                                 const math::Vector3 &_angular,
                                 bool _updateChildren = true)
                                 GAZEBO_DEPRECATED(7.0);

      /// \brief Set angular and linear rates of an physics::Entity.
      /// \param[in] _linear Linear twist.
      /// \param[in] _angular Angular twist.
      /// \param[in] _updateChildren True to pass this update to child
      /// entities.
      public: void SetWorldTwist(const ignition::math::Vector3d &_linear,
                                 const ignition::math::Vector3d &_angular,
                                 const bool _updateChildren = true);

      /// \brief Returns Entity#dirtyPose.
      ///
      /// The dirty pose is the pose set by the physics engine before it's
      /// value is propagated to the rest of the simulator.
      /// \return The dirty pose of the entity.
      /// \deprecated See DirtyPose()
      public: math::Pose GetDirtyPose() const GAZEBO_DEPRECATED(7.0);

      /// \brief Returns Entity#dirtyPose.
      ///
      /// The dirty pose is the pose set by the physics engine before it's
      /// value is propagated to the rest of the simulator.
      /// \return The dirty pose of the entity.
      public: const ignition::math::Pose3d &DirtyPose() const;

      /// \brief This function is called when the entity's
      /// (or one of its parents) pose of the parent has changed.
      protected: virtual void OnPoseChange() = 0;

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to protected data
      /// \param[in] _parent Parent of the entity.
      protected: Entity(EntityPrivate &_dataPtr, BasePtr _parent);

      /// \brief Shared construction code.
      private: void ConstructionHelper();

      /// \brief Publish the pose.
      private: virtual void PublishPose();

      /// \brief Helper function to get the collision bounding box.
      /// \param[in] _base Object to calculated the bounding box for.
      /// \return The boundin box for the passed in object.
      private: ignition::math::Box CollisionBoundingBoxHelper(
                   BasePtr _base) const;

      /// \brief Set the world pose for a model.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseModel(const ignition::math::Pose3d &_pose,
                                      const bool _notify,
                                      const bool _publish);

      /// \brief Set the world pose for a canonical Link.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseCanonicalLink(
                   const ignition::math::Pose3d &_pose,
                   const bool _notify, const bool _publish);

      /// \brief Set the world pose for a common entity.
      /// \param[in] _pose New pose for the entity.
      /// \param[in] _notify True to notify children of the pose update.
      /// \param[in] _publish True to publish the pose.
      private: void SetWorldPoseDefault(
                   const ignition::math::Pose3d &_pose,
                   const bool _notify, const bool _publish);

      /// \brief Called when a new pose message arrives.
      /// \param[in] _msg The message to set the pose from.
      private: void OnPoseMsg(ConstPosePtr &_msg);

      /// \brief Handle a change of pose
      /// \param[in] update_children if set to true, will call OnPoseChange
      ///            for all children (1 level, non-recursive).
      ///            But if the Object is static, we force children
      ///            OnPoseChange call
      private: void UpdatePhysicsPose(const bool update_children = true);

      /// \brief Update an animation.
      /// \param[in] _info Update information.
      private: void UpdateAnimation(const common::UpdateInfo &_info);

      /// \internal
      /// \brief Private data pointer
      protected: EntityPrivate *entityDPtr;
    };
    /// \}
  }
}
#endif
