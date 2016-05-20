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
#ifndef GAZEBO_PHYSICS_ACTOR_HH_
#define GAZEBO_PHYSICS_ACTOR_HH_

#include <string>
#include <map>

#include "gazebo/physics/Model.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Color;
  }

  namespace physics
  {
    // Forward declare private data.
    class ActorPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Actor Actor.hh physics/physics.hh
    /// \brief Actor class enables GPU based mesh model / skeleton
    /// scriptable animation.
    class GZ_PHYSICS_VISIBLE Actor : public Model
    {
      /// \brief Typedef the skeleton animation map.
      public: typedef std::map<std::string, common::SkeletonAnimation*>
              SkeletonAnimation_M;

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
      public: virtual bool IsActive() const;

      /// \brief Update the actor
      public: void Update();

      /// \brief Finalize the actor
      public: virtual void Fini();

      /// \brief update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the current script time.
      /// \param[in] _time Current script time.
      public: void SetScriptTime(const double _time);

      /// \brief Get the current script time.
      /// \return Script time.
      public: double ScriptTime() const;

      /// \brief Returns a dictionary of all the skeleton animations associated
      /// with the actor.
      /// \return a map of SkeletonAnimation, indexed by their name.
      public: const SkeletonAnimation_M &SkeletonAnimations() const;

      /// \brief Set a custom trajectory for the actor.
      /// \param[in] _trajInfo Information about custom trajectory.
      public: void SetCustomTrajectory(TrajectoryInfoPtr &_trajInfo);

      /// \brief Reset custom trajectory of the actor.
      public: void ResetCustomTrajectory();

      // Documentation inherited
      public: virtual bool GetSelfCollide() const;

      // Documentation inherited
      public: virtual void SetSelfCollide(bool _self_collide);

      /// \brief Add inertia for a sphere.
      /// \param[in] _linkSdf The link to add the inertia to.
      /// \param[in] _pose Pose of the inertia.
      /// \param[in] _mass Mass of the inertia.
      /// \param[in] _radius Radius of the sphere.
      private: void AddSphereInertia(const sdf::ElementPtr &_linkSdf,
                   const ignition::math::Pose3d &_pose,
                   const double _mass, const double _radius);

      /// \brief Add a spherical collision object.
      /// \param[in] _linkSdf Link to add the collision to.
      /// \param[in] _name Name of the collision object.
      /// \param[in] _pose Pose of the collision object.
      /// \param[in] _radius Radius of the collision object.
      private: void AddSphereCollision(const sdf::ElementPtr &_linkSdf,
                   const std::string &_name,
                   const ignition::math::Pose3d &_pose,
                   const double _radius);

      /// \brief Add a spherical visual object.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual object.
      /// \param[in] _pose Pose of the visual object.
      /// \param[in] _radius Radius of the visual object.
      /// \param[in] _material Name of the visual material.
      /// \param[in] _ambient Ambient color.
      private: void AddSphereVisual(const sdf::ElementPtr &_linkSdf,
                   const std::string &_name,
                   const ignition::math::Pose3d &_pose, const double _radius,
                   const std::string &_material, const common::Color &_ambient);

      /// \brief Add a box visual object.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual object.
      /// \param[in] _pose Pose of the visual object.
      /// \param[in] _size Dimensions of the visual object.
      /// \param[in] _material Name of the visual material.
      /// \param[in] _ambient Ambient color.
      private: void AddBoxVisual(const sdf::ElementPtr &_linkSdf,
                   const std::string &_name,
                   const ignition::math::Pose3d &_pose,
                   const ignition::math::Vector3d &_size,
                   const std::string &_material,
                   const common::Color &_ambient);

      /// \brief Add an actor visual to a link.
      /// \param[in] _linkSdf Link to add the visual to.
      /// \param[in] _name Name of the visual.
      /// \param[in] _pose Pose of the visual.
      private: void AddActorVisual(const sdf::ElementPtr &_linkSdf,
                   const std::string &_name,
                   const ignition::math::Pose3d &_pose);

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
      private: void SetPose(
                   const std::map<std::string,
                                  ignition::math::Matrix4d> &_frame,
                   const std::map<std::string,
                                  std::string> &_skelMap,
                  const double _time);

      /// \internal
      /// \brief Private data pointer.
      private: ActorPrivate *actorDPtr;
    };
    /// \}
  }
}
#endif
