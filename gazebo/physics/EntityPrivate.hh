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
#ifndef GAZEBO_PHYSICS_ENTITY_PRIVATE_HH_
#define GAZEBO_PHYSICS_ENTITY_PRIVATE_HH_

#include <functional>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/physics/BasePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Entity private data.
    class EntityPrivate : public BasePrivate
    {
      /// \brief A helper that prevents numerous dynamic_casts.
      public: EntityPtr parentEntity;

      /// \brief World pose of the entity.
      public: mutable ignition::math::Pose3d worldPose;

      /// \brief Communication node.
      public: transport::NodePtr node;

      /// \brief Visual publisher.
      public: transport::PublisherPtr visPub;

      /// \brief Pose publisher.
      public: transport::PublisherPtr posePub;

      /// \brief Request publisher.
      public: transport::PublisherPtr requestPub;

      /// \brief Visual message container.
      public: msgs::Visual *visualMsg;

      /// \brief Current pose animation
      public: common::PoseAnimationPtr animation;

      /// \brief Previous time an animation was updated.
      public: common::Time prevAnimationTime;

      /// \brief Start pose of an animation.
      public: ignition::math::Pose3d animationStartPose;

      /// \brief All our event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Connection used to update an animation.
      public: event::ConnectionPtr animationConnection;

      /// \brief The pose set by a physics engine.
      public: ignition::math::Pose3d dirtyPose;

      /// \brief Scale of the entity
      public: ignition::math::Vector3d scale;

      /// \brief True if the object is static.
      public: bool isStatic;

      /// \brief Only used by Links. Included here for performance.
      public: bool isCanonicalLink;

      /// \brief The initial pose of the entity.
      public: ignition::math::Pose3d initialRelativePose;

      /// \brief Pose subscriber.
      public: transport::SubscriberPtr poseSub;

      /// \brief Callback for when an animation completes.
      public: std::function<void()> onAnimationComplete;

      /// \brief The function used to to set the world pose.
      public: std::function<void(const ignition::math::Pose3d &,
                  const bool, const bool)> setWorldPoseFunc;
      /*public: void (Entity::*setWorldPoseFunc)(
                   const ignition::math::Pose3d &, const bool, const bool);
                   */
    };
  }
}
#endif
