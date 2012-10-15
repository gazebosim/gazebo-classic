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
/* Desc: Base class for all physical entities
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#ifndef ENTITY_HH
#define ENTITY_HH

#include <string>
#include <vector>

#include "msgs/msgs.hh"

#include "transport/TransportTypes.hh"
#include "common/CommonTypes.hh"
#include "math/MathTypes.hh"
#include "math/Box.hh"

#include "math/Pose.hh"
#include "physics/PhysicsTypes.hh"

#include "physics/Base.hh"

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

    /// \brief Base class for all physics objects in Gazebo
    class Entity : public Base
    {
      /// \brief Constructor
      /// \param parent Parent of the entity.
      public: Entity(BasePtr parent);

      /// \brief Destructor
      public: virtual ~Entity();

      /// \brief Load
      /// \param node Pointer to an configuration node
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Finalize the entity
      public: virtual void Fini();

      public: virtual void Reset();

      /// \brief Update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the name of the entity
      /// \param name The new name
      public: virtual void SetName(const std::string &name);

      /// \brief Set whether this entity is static: immovable
      /// \param s Bool, true = static
      public: void SetStatic(const bool &s);

      /// \brief Return whether this entity is static
      /// \return bool True = static
      public: bool IsStatic() const;

      /// \brief Set the initial pose
      /// \param p The initial pose
      public: void SetInitialRelativePose(const math::Pose &p);

      /// \brief Return the bounding box for the entity
      public: virtual math::Box GetBoundingBox() const;

      /// \brief Get the absolute pose of the entity
      public: inline const math::Pose &GetWorldPose() const
              {return this->worldPose;}

      /// \brief Get the pose of the entity relative to its parent
      public: math::Pose GetRelativePose() const;

      /// \brief Set the pose of the entity relative to its parent
      /// \param pose The new pose
      /// \param notify True = tell children of the pose change
      public: void SetRelativePose(const math::Pose &pose, bool notify = true,
                                   bool publish = true);

      /// \brief Set the world pose of the entity
      /// \param pose The new world pose
      /// \param notify True = tell children of the pose change
      public: void SetWorldPose(const math::Pose &pose, bool notify = true,
                                bool publish = true);

      /// \brief Get the linear velocity of the entity
      /// \return A math::Vector3 for the linear velocity
      public: virtual math::Vector3 GetRelativeLinearVel() const
              {return math::Vector3();}
      /// \brief Get the linear velocity of the entity in the world frame
      /// \return A math::Vector3 for the linear velocity
      public: virtual math::Vector3 GetWorldLinearVel() const
              {return math::Vector3();}
      /// \brief Get the angular velocity of the entity
      /// \return A math::Vector3 for the velocity
      public: virtual math::Vector3 GetRelativeAngularVel() const
              {return math::Vector3();}
      /// \brief Get the angular velocity of the entity in the world frame
      /// \return A math::Vector3 for the velocity
      public: virtual math::Vector3 GetWorldAngularVel() const
              {return math::Vector3();}
      /// \brief Get the linear acceleration of the entity
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetRelativeLinearAccel() const
              {return math::Vector3();}
      /// \brief Get the linear acceleration of the entity in the world frame
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetWorldLinearAccel() const
              {return math::Vector3();}

      /// \brief Get the angular acceleration of the entity
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetRelativeAngularAccel() const
              {return math::Vector3();}
      /// \brief Get the angular acceleration of the entity in the world frame
      /// \return A math::Vector3 for the acceleration
      public: virtual math::Vector3 GetWorldAngularAccel() const
              {return math::Vector3();}
      /// \brief Set to true if this entity is a canonical link for a model.
      /// \param _value True if the link is canonical.
      public: void SetCanonicalLink(bool _value);

      /// \brief A helper function that checks if this is a canonical body
      public: inline bool IsCanonicalLink() const
              { return this->isCanonicalLink; }
      /// \brief Set an animation for this entity
      public: void SetAnimation(const common::PoseAnimationPtr &_anim,
                                boost::function<void()> _onComplete);
      /// \brief Set an animation for this entity
      public: void SetAnimation(common::PoseAnimationPtr _anim);

      /// \brief Stop the current animation, if any
      public: virtual void StopAnimation();

      private: void PublishPose();

      /// \brief Get the parent model, if one exists
      /// \return Pointer to a model, or NULL if no parent model exists
      public: ModelPtr GetParentModel();

      /// \brief Get a child collision entity, if one exists
      public: CollisionPtr GetChildCollision(const std::string &_name);

      /// \brief Get a child linke entity, if one exists
      public: LinkPtr GetChildLink(const std::string &_name);

      /// \brief Get the distance to the nearest entity below
      ///        (along the Z-axis) this entity.
      /// \param _distBelow The distance to the nearest entity below
      /// \param _entityName The name of the nearest entity below
      public: void GetNearestEntityBelow(double &_distBelow,
                                         std::string &_entityName);

      /// \brief Move this entity to be ontop of the nearest entity below
      public: void PlaceOnNearestEntityBelow();

      /// \brief Move this entity to be ontop of another entity by name
      public: void PlaceOnEntity(const std::string &_entityName);

      /// \brief Returns collision bounding box
      public: math::Box GetCollisionBoundingBox() const;

      /// \brief Set angular and linear rates of an physics::Entity
      public: void SetWorldTwist(const math::Vector3 &linear,
                                 const math::Vector3 &angular,
                                 bool updateChildren = true);

      /// \brief Returns Entity#dirtyPose
      public: const math::Pose &GetDirtyPose() const;

      private: math::Box GetCollisionBoundingBoxHelper(BasePtr _base) const;

      private: void SetWorldPoseModel(const math::Pose &_pose, bool _notify,
                                      bool _publish);

      private: void SetWorldPoseCanonicalLink(const math::Pose &_pose,
                                              bool _notify, bool _publish);

      private: void SetWorldPoseDefault(const math::Pose &_pose, bool _notify,
                                        bool _publish);

      /// \brief Called when a new pose message arrives
      private: void OnPoseMsg(ConstPosePtr &_msg);

      /// \brief This function is called when the entity's
      ///        (or one of its parents) pose of the parent has changed
      protected: virtual void OnPoseChange() = 0;

      /// \brief Handle a change of pose
      /// \param[in] update_children if set to true, will call OnPoseChange
      ///            for all children (1 level, non-recursive).
      ///            But if the Object is static, we force children
      ///            OnPoseChange call
      private: void UpdatePhysicsPose(bool update_children = true);

      /// \brief Update an animation
      private: void UpdateAnimation();

      /// A helper that prevents numerous dynamic_casts
      protected: EntityPtr parentEntity;

      private: bool isStatic;

      /// \brief Only used by Links. Included here for performance.
      private: bool isCanonicalLink;

      /// The initial pose of the entity
      private: math::Pose initialRelativePose;
      private: math::Pose worldPose;

      protected: transport::NodePtr node;
      private: transport::PublisherPtr posePub;
      private: transport::SubscriberPtr poseSub;
      protected: transport::PublisherPtr visPub;
      protected: transport::PublisherPtr requestPub;

      protected: msgs::Visual *visualMsg;
      protected: msgs::Pose *poseMsg;

      protected: common::PoseAnimationPtr animation;
      protected: common::Time prevAnimationTime;
      protected: math::Pose animationStartPose;

      protected: std::vector<event::ConnectionPtr> connections;
      protected: event::ConnectionPtr animationConnection;

      protected: math::Pose dirtyPose;
      private: boost::function<void()> onAnimationComplete;

      private: void (Entity::*setWorldPoseFunc)(const math::Pose &, bool, bool);
    };

    /// \}
  }
}
#endif


