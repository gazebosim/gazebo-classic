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
#include <string>
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
      public: const common::Mesh *mesh;

      /// \brief The actor's skeleton.
      public: common::Skeleton *skeleton;

      /// \brief Filename for the skin.
      public: std::string skinFile;

      /// \brief Scaling factor to apply to the skin.
      public: double skinScale;

      /// \brief Amount of time to delay start by.
      public: double startDelay;

      /// \brief Time length of a scipt.
      public: double scriptLength;

      /// \brief Time the scipt was last updated.
      public: double lastScriptTime;

      /// \brief True if the animation should loop.
      public: bool loop;

      /// \brief True if the actor is being updated.
      public: bool active;

      /// \brief True if the actor should start running automatically.
      public: bool autoStart;

      /// \brief Base link.
      public: LinkPtr mainLink;

      /// \brief Time of the previous frame.
      public: common::Time prevFrameTime;

      /// \brief Time when the animation was started.
      public: common::Time playStartTime;

      /// \brief All the trajectories.
      public: std::map<unsigned int, common::PoseAnimation*> trajectories;

      /// \brief Trajectory information
      public: std::vector<TrajectoryInfo> trajInfo;

      /// \brief Skeleton animations
      public: std::map<std::string, common::SkeletonAnimation*>
                                                            skelAnimation;

      /// \brief Skeleton to naode map
      public: std::map<std::string, std::map<std::string, std::string> >
                                                            skelNodesMap;

      /// \brief True to interpolate along x direction.
      public: std::map<std::string, bool> interpolateX;

      /// \brief Last position of the actor
      public: ignition::math::Vector3d lastPos;

      /// \brief Length of the actor's path.
      public: double pathLength;

      /// \brief THe last trajectory
      public: unsigned int lastTraj;

      /// \brief Name of the visual
      public: std::string visualName;

      /// \brief ID for this visual
      public: uint32_t visualId;

      /// \brief Where to send bone info.
      public: transport::PublisherPtr bonePosePub;

      /// \brief Current script time.
      public: double scriptTime;

      /// \brief Custom trajectory.
      /// Used to control an actor with a plugin.
      public: TrajectoryInfoPtr customTrajectoryInfo;
    };
  }
}
#endif
