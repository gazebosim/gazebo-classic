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
#ifndef GAZEBO_PHYSICS_ACTORPRIVATE_HH_
#define GAZEBO_PHYSICS_ACTORPRIVATE_HH_

#include <map>
#include <vector>

#include "gazebo/common/Time.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/physics/ModelPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Information about a trajectory for an Actor.
    class TrajectoryInfo
    {
      /// \brief Constructor.
      public: TrajectoryInfo();

      /// \brief ID of the trajectory.
      public: unsigned int id;

      /// \brief Type of trajectory.
      public: std::string type;

      /// \brief Duration of the trajectory.
      public: double duration;

      /// \brief Start time of the trajectory.
      public: double startTime;

      /// \brief End time of the trajectory.
      public: double endTime;

      /// \brief True if the trajectory is tranlated.
      public: bool translated;
    };

    class ActorPrivate : public ModelPrivate
    {
      /// \brief Pointer to the actor's mesh.
      public: const common::Mesh *mesh = nullptr;

      /// \brief The actor's skeleton.
      public: common::Skeleton *skeleton = nullptr;

      /// \brief Filename for the skin.
      public: std::string skinFile;

      /// \brief Scaling factor to apply to the skin.
      public: double skinScale;

      /// \brief Time to wait before starting the script. If running in a loop,
      /// this time will be waited before starting each cycle.
      public: double startDelay;

      /// \brief Total time length of the script, in seconds.
      public: double scriptLength;

      /// \brief True if the animation should loop.
      public: bool loop;

      /// \brief True if the actor is being updated.
      public: bool active;

      /// \brief True if the actor should start running automatically,
      /// otherwise it will only start once Play is called.
      public: bool autoStart;

      /// \brief Pointer to the actor's canonical link.
      public: LinkPtr mainLink;

      /// \brief Time of the previous frame.
      public: common::Time prevFrameTime;

      /// \brief Time when the animation was started.
      public: common::Time playStartTime;

      /// \brief Map of all the trajectories (pose animations) and their
      /// indices. The indices here match the order in `trajInfo`.
      /// \sa trajInfo
      public: std::map<unsigned int, common::PoseAnimation*> trajectories;

      /// \brief A vector of trajectory information, which contains information
      /// such as their durations, uniquely identifiable by their IDs. The IDs
      /// here match those on the `trajectories` vector.
      /// \sa trajectories
      public: std::vector<TrajectoryInfo> trajInfo;

      /// \brief Map of skeleton animations, indexed by their names. The names
      /// match those in `interpolateX` and `skelNodesMap`.
      /// \sa interpolateX
      /// \sa skelNodesMap
      public: common::SkeletonAnimation_M skelAnimation;

      /// \brief Skeleton to node map:
      /// * Skeleton animation name (should match those in `skelAnimation` and
      /// `interpolateX`)
      /// * Map holding:
      ///     * Skeleton node names from skin
      ///     * Skeleton node names from animation
      /// \sa interpolateX
      /// \sa skelAnimation
      public: std::map<std::string, std::map<std::string, std::string> >
                                                            skelNodesMap;

      /// \brief Map of animation types (the same name as in `skelAnimation` and
      /// `skelNodesMap`) and whether they should be interpolated along X
      // direction.
      /// \sa skelAnimation
      /// \sa skelNodesMap
      public: std::map<std::string, bool> interpolateX;

      /// \brief Last position of the actor.
      public: ignition::math::Vector3d lastPos;

      /// \brief Length of the actor's path.
      public: double pathLength;

      /// \brief Id of the last trajectory
      public: unsigned int lastTraj;

      /// \brief Name of the visual representing the skin.
      public: std::string visualName;

      /// \brief ID for the visual representing the skin.
      public: uint32_t visualId;

      /// \brief Publisher to send bone info.
      public: transport::PublisherPtr bonePosePub;

      /// \brief Current time within the script, which is the current time minus
      /// the time when the script started.
      public: double scriptTime;

      /// \brief Custom trajectory.
      /// Used to control an actor with a plugin.
      public: TrajectoryInfoPtr customTrajectoryInfo;
    };
  }
}
#endif
