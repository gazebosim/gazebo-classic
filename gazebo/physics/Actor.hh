/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _ACTOR_HH_
#define _ACTOR_HH_

#include <string>
#include <map>
#include <vector>

#include "gazebo/physics/Model.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Mesh;
    class Color;
  }

  namespace physics
  {
    /// \brief Information about a trajectory for an Actor.
    class GAZEBO_VISIBLE TrajectoryInfo
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

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Actor Actor.hh physics/physics.hh
    /// \brief Actor class enables GPU based mesh model / skeleton
    /// scriptable animation.
    class GAZEBO_VISIBLE Actor : public Model
    {
      /// \brief Constructor
      /// \param[in] _parent Parent object
      public: explicit Actor(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Actor();

      /// \brief Load the actor
      /// \param[in] _sdf SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the actor
      public: virtual void Init();

      /// \brief Start playing the script
      public: virtual void Play();

      /// \brief Stop playing the script
      public: virtual void Stop();

      /// \brief Returns true when actor is playing animation
      public: virtual bool IsActive();

      /// \brief Update the actor
      public: void Update();

      /// \brief Finalize the actor
      public: virtual void Fini();

      /// \brief update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Get the SDF values for the actor.
      /// \return Pointer to the SDF values.
      public: virtual const sdf::ElementPtr GetSDF();

      /// \brief Add inertia for a sphere.
      /// \param[in] _linkSdf The link to add the inertia to.
      /// \param[in] _pose Pose of the inertia.
      /// \param[in] _mass Mass of the inertia.
      /// \param[in] _radiau Radius of the sphere.
      private: void AddSphereInertia(sdf::ElementPtr _linkSdf,
                                     const math::Pose &_pose,
                                     double _mass, double _radius);

      /// \brief Add a spherical collision object.
      /// \param[in] _linkSdf Link to add the collision to.
      /// \param[in] _name Name of the collision object.
      /// \param[in] _pose Pose of the collision object.
      /// \param[in] _radius Radius of the collision object.
      private: void AddSphereCollision(sdf::ElementPtr _linkSdf,
                                       const std::string &_name,
                                       const math::Pose &_pose,
                                       double _radius);

      /// \brief Add a spherical visual object.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual object.
      /// \param[in] _pose Pose of the visual object.
      /// \param[in] _radius Radius of the visual object.
      /// \param[in] _material Name of the visual material.
      /// \param[in] _ambient Ambient color.
      private: void AddSphereVisual(sdf::ElementPtr _linkSdf,
                                    const std::string &_name,
                                    const math::Pose &_pose,
                                    double _radius,
                                    const std::string &_material,
                                    const common::Color &_ambient);

      /// \brief Add a box visual object.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual object.
      /// \param[in] _pose Pose of the visual object.
      /// \param[in] _size Dimensions of the visual object.
      /// \param[in] _material Name of the visual material.
      /// \param[in] _ambient Ambient color.
      private: void AddBoxVisual(sdf::ElementPtr _linkSdf,
                                 const std::string &_name,
                                 const math::Pose &_pose,
                                 const math::Vector3 &_size,
                                 const std::string &_material,
                                 const common::Color &_ambient);

      /// \brief Add an actor visual to a link.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual.
      /// \param[in] _pose Pose of the visual.
      private: void AddActorVisual(sdf::ElementPtr _linkSdf,
                                   const std::string &_name,
                                   const math::Pose &_pose);

      /// \brief Load an animation from SDF.
      /// \param[in] _sdf SDF element containing the animation.
      private: void LoadAnimation(sdf::ElementPtr _sdf);

      /// \brief Load an animation script from SDF.
      /// \param[in] _sdf SDF element containing the animation script.
      private: void LoadScript(sdf::ElementPtr _sdf);

      /// \brief Set the actor's pose.
      /// \param[in] _frame Each frame name and transform.
      /// \param[in] _skelMap Map of bone relationships.
      /// \param[in] _time Time over which to animate the set pose.
      private: void SetPose(std::map<std::string, math::Matrix4> _frame,
                     std::map<std::string, std::string> _skelMap, double _time);

      /// \brief Pointer to the actor's mesh.
      protected: const common::Mesh *mesh;

      /// \brief The actor's skeleton.
      protected: common::Skeleton *skeleton;

      /// \brief Filename for the skin.
      protected: std::string skinFile;

      /// \brief Scaling factor to apply to the skin.
      protected: double skinScale;

      /// \brief Amount of time to delay start by.
      protected: double startDelay;

      /// \brief Time length of a scipt.
      protected: double scriptLength;

      /// \brief Time the scipt was last updated.
      protected: double lastScriptTime;

      /// \brief True if the animation should loop.
      protected: bool loop;

      /// \brief True if the actor is being updated.
      protected: bool active;

      /// \brief True if the actor should start running automatically.
      protected: bool autoStart;

      /// \brief Base link.
      protected: LinkPtr mainLink;

      /// \brief Time of the previous frame.
      protected: common::Time prevFrameTime;

      /// \brief Time when the animation was started.
      protected: common::Time playStartTime;

      /// \brief All the trajectories.
      protected: std::map<unsigned int, common::PoseAnimation*> trajectories;

      /// \brief Trajectory information
      protected: std::vector<TrajectoryInfo> trajInfo;

      /// \brief Skeleton animations
      protected: std::map<std::string, common::SkeletonAnimation*>
                                                            skelAnimation;

      /// \brief Skeleton to naode map
      protected: std::map<std::string, std::map<std::string, std::string> >
                                                            skelNodesMap;

      /// \brief True to interpolate along x direction.
      protected: std::map<std::string, bool> interpolateX;

      /// \brief Last position of the actor
      protected: math::Vector3 lastPos;

      /// \brief Length of the actor's path.
      protected: double pathLength;

      /// \brief THe last trajectory
      protected: unsigned int lastTraj;

      /// \brief Name of the visual
      protected: std::string visualName;

      /// \brief ID for this visual
      protected: uint32_t visualId;

      /// \brief Where to send bone info.
      protected: transport::PublisherPtr bonePosePub;

      /// \brief THe old action.
      protected: std::string oldAction;
    };
    /// \}
  }
}
#endif
