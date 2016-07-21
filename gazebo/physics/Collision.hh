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
#ifndef GAZEBO_PHYSICS_COLLISION_HH_
#define GAZEBO_PHYSICS_COLLISION_HH_

#include "gazebo/physics/CollisionState.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class.
    class CollisionPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for all collision entities
    class GZ_PHYSICS_VISIBLE Collision : public Entity
    {
      /// \brief Constructor.
      /// \param[in] _link Link that contains this collision object.
      public: explicit Collision(LinkPtr _link);

      /// \brief Destructor.
      public: virtual ~Collision();

      /// \brief Finalize the collision.
      public: virtual void Fini();

      /// \brief Load the collision.
      /// \param[in] _sdf SDF to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the collision.
      public: virtual void Init();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the encapsulated collsion object.
      /// \param[in] _placeable True to make the object movable.
      public: void SetCollision(const bool _placeable);

      /// \brief Return whether this collision is movable.
      /// Example on an immovable object is a ray.
      /// \return True if the object is immovable.
      public: bool IsPlaceable() const;

      /// \brief Set the category bits, used during collision detection.
      /// \param[in] _bits The bits to set.
      public: virtual void SetCategoryBits(const unsigned int _bits) = 0;

      /// \brief Set the collide bits, used during collision detection.
      /// \param[in] _bits The bits to set.
      public: virtual void SetCollideBits(const unsigned int _bits) = 0;

      /// \brief Set the laser retro reflectiveness.
      /// \param[in] _retro The laser retro value.
      public: void SetLaserRetro(const float _retro);

      /// \brief Get the laser retro reflectiveness
      /// \return The laser retro value.
      /// \deprecated See LaserRetro()
      public: float GetLaserRetro() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the laser retro reflectiveness
      /// \return The laser retro value.
      public: float LaserRetro() const;

      /// \brief Get the link this collision belongs to.
      /// \return The parent Link.
      /// \deprecated See Link();
      public: LinkPtr GetLink() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the link this collision belongs to.
      /// \return The parent Link.
      /// \deprecated See Link();
      public: LinkPtr Link() const;

      /// \brief Get the model this collision belongs to.
      /// \return The parent model.
      /// \deprecated See ParentModel() const
      public: ModelPtr GetModel() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the model this collision belongs to.
      /// \return The parent model.
      public: Model *ParentModel() const;

      // Documentation inherited
      public: virtual ignition::math::Box BoundingBox() const = 0;

      /// \brief Get the shape type.
      /// \return The shape type.
      /// \sa EntityType
      /// \deprecated See ShapeType() const
      public: unsigned int GetShapeType() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the shape type.
      /// \return The shape type.
      /// \sa EntityType
      public: unsigned int ShapeType() const;

      /// \brief Set the shape for this collision.
      /// \param[in] _shape The shape for this collision object.
      public: void SetShape(ShapePtr _shape);

      /// \brief Get the collision shape.
      /// \return The collision shape.
      /// \deprecated See Shape() const
      public: ShapePtr GetShape() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the collision shape.
      /// \return The collision shape.
      public: ShapePtr Shape() const;

      /// \brief Set the scale of the collision.
      /// \param[in] _scale Scale to set the collision to.
      public: void SetScale(const math::Vector3 &_scale);

      /// \brief Get the linear velocity of the collision.
      /// \return The linear velocity relative to the parent model.
      /// \deprecated See GetRelativeLinearVel() const
      public: virtual math::Vector3 GetRelativeLinearVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear velocity of the collision.
      /// \return The linear velocity relative to the parent model.
      public: virtual ignition::math::Vector3d RelativeLinearVel() const;

      /// \brief Get the linear velocity of the collision in the world
      /// frame.
      /// \return The linear velocity of the collision in the world frame.
      /// \deprecated See WorldLinearVel() const
      public: virtual math::Vector3 GetWorldLinearVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear velocity of the collision in the world
      /// frame.
      /// \return The linear velocity of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldLinearVel() const;

      /// \brief Get the angular velocity of the collision.
      /// \return The angular velocity of the collision.
      /// \deprecated See RelativeAngularVel() const
      public: virtual math::Vector3 GetRelativeAngularVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular velocity of the collision.
      /// \return The angular velocity of the collision.
      public: virtual ignition::math::Vector3d RelativeAngularVel() const;

      /// \brief Get the angular velocity of the collision in the world frame.
      /// \return The angular velocity of the collision in the world frame.
      /// \deprecated See GetWorldAngularVel() const
      public: virtual math::Vector3 GetWorldAngularVel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular velocity of the collision in the world frame.
      /// \return The angular velocity of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      /// \brief Get the linear acceleration of the collision.
      /// \return The linear acceleration of the collision.
      /// \deprecated See RelativeLinearAccel();
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear acceleration of the collision.
      /// \return The linear acceleration of the collision.
      public: virtual ignition::math::Vector3d RelativeLinearAccel() const;

      /// \brief Get the linear acceleration of the collision in the world
      /// frame.
      /// \return The linear acceleration of the collision in the world frame.
      /// \deprecated See tWorldLinearAccel() const
      public: virtual math::Vector3 GetWorldLinearAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the linear acceleration of the collision in the world
      /// frame.
      /// \return The linear acceleration of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldLinearAccel() const;

      /// \brief Get the angular acceleration of the collision.
      /// \return The angular acceleration of the collision.
      /// \deprecated See RelativeAngularAccel() const
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular acceleration of the collision.
      /// \return The angular acceleration of the collision.
      public: virtual ignition::math::Vector3d RelativeAngularAccel() const;

      /// \brief Get the angular acceleration of the collision in the
      /// world frame.
      /// \return The angular acceleration of the collision in the world frame.
      /// \deprecated See GetWorldAngularAccel() const
      public: virtual math::Vector3 GetWorldAngularAccel() const
              GAZEBO_DEPRECATED(7.0);

      /// \brief Get the angular acceleration of the collision in the
      /// world frame.
      /// \return The angular acceleration of the collision in the world frame.
      public: virtual ignition::math::Vector3d WorldAngularAccel() const;

      /// \brief Get the collision state.
      /// \return The collision state.
      /// \deprecated See State() const
      public: CollisionState GetState() GAZEBO_DEPRECATED(7.0);

      /// \brief Get the collision state.
      /// \return The collision state.
      public: CollisionState State() const;

      /// \brief Set the current collision state.
      /// \param[in] The collision state.
      public: void SetState(const CollisionState &_state);

      /// \brief Fill a collision message.
      /// \param[out] _msg The message to fill with this collision's data.
      public: void FillMsg(msgs::Collision &_msg);

      /// \brief Update parameters from a message.
      /// \param[in] _msg Message to update from.
      public: void ProcessMsg(const msgs::Collision &_msg);

      /// \brief Get the surface parameters.
      /// \return The surface parameters.
      /// \deprecated See Surface() const
      public: SurfaceParamsPtr GetSurface() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the surface parameters.
      /// \return The surface parameters.
      public: SurfaceParamsPtr Surface() const;

      /// \brief Number of contacts allowed for this collision.
      /// This overrides global value (in PhysicsEngine) if specified.
      /// \param[in] _maxContacts max num contacts allowed for this collision.
      public: virtual void SetMaxContacts(const unsigned int _maxContacts);

      /// \brief returns number of contacts allowed for this collision.
      /// This overrides global value (in PhysicsEngine) if specified.
      /// \return max num contacts allowed for this collision.
      /// \deprecated See MaxContacts() const
      public: virtual unsigned int GetMaxContacts() GAZEBO_DEPRECATED(7.0);

      /// \brief returns number of contacts allowed for this collision.
      /// This overrides global value (in PhysicsEngine) if specified.
      /// \return max num contacts allowed for this collision.
      public: virtual unsigned int MaxContacts() const;

      /// \brief Indicate that the world pose should be recalculated.
      /// The recalculation will be done when Collision::GetWorldPose is
      /// called.
      public: void SetWorldPoseDirty();

      // Documentation inherited.
      public: virtual const ignition::math::Pose3d &WorldPose() const;

      /// \internal
      /// \brief Constructor used by inherited classes
      /// \param[in] _dataPtr Pointer to protected data
      /// \param[in] _link Pointer to parent link
      protected: Collision(CollisionPrivate &_dataPtr, LinkPtr _link);

      /// \brief Shared construction code.
      /// \param[in] _link Pointer to parent link
      private: void ConstructionHelper(LinkPtr _Link);

      /// \brief Helper function used to create a collision visual message.
      /// \return Visual message for a collision.
      private: msgs::Visual CreateCollisionVisual();

      /// \internal
      /// \brief Protected data pointer
      protected: CollisionPrivate *collDPtr;
    };
    /// \}
  }
}
#endif
