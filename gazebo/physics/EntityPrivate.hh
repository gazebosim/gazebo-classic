/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_ENTITY_PRIVATE_HH_
#define _GAZEBO_PHYSICS_ENTITY_PRIVATE_HH_

#include <ignition/math/Pose3d.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/physics/BasePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Entity protected data.
    class EntityProtected : public BaseProtected
    {
      /// \brief A helper that prevents numerous dynamic_casts.
      protected: EntityPtr parentEntity;

      /// \brief World pose of the entity.
      protected: mutable ignition::math::Pose3d worldPose;

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
      protected: ignition::math::Pose3d animationStartPose;

      /// \brief All our event connections.
      protected: std::vector<event::ConnectionPtr> connections;

      /// \brief Connection used to update an animation.
      protected: event::ConnectionPtr animationConnection;

      /// \brief The pose set by a physics engine.
      protected: ignition::math::Pose3d dirtyPose;

      /// \brief Scale of the entity
      protected: ignition::math::Vector3d scale;
    };

    /// \internal
    /// \brief Entity protected data.
    class EntityPrivate : public EntityProtected
    {
      /// \brief True if the object is static.
      private: bool isStatic;

      /// \brief Only used by Links. Included here for performance.
      private: bool isCanonicalLink;

      /// \brief The initial pose of the entity.
      private: ignition::math::Pose3d initialRelativePose;

      /// \brief Pose subscriber.
      private: transport::SubscriberPtr poseSub;

      /// \brief Callback for when an animation completes.
      private: boost::function<void()> onAnimationComplete;

      /// \brief The function used to to set the world pose.
      private: void (Entity::*setWorldPoseFunc)(
                   const ignition::math::Pose3d &, const bool, const bool);
    };
  }
}
#endif
