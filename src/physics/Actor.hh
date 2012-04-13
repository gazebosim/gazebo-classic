/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Base class for animated models
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */

#ifndef ACTOR_HH
#define ACTOR_HH

#include <string>

#include "physics/Model.hh"
#include "common/Time.hh"
#include "common/Skeleton.hh"

namespace gazebo
{
  namespace common
  {
    class Mesh;
    class Color;
  }
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    /// \brief An actor
    class Actor : public Model
    {
      /// \brief Constructor
      /// \param parent Parent object
      public: Actor(BasePtr parent);

      /// \brief Destructor
      public: virtual ~Actor();

      /// \brief Load the actor
      /// \param _sdf SDF parameters
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the actor
      public: virtual void Init();

      /// \brief Update the actor
      public: void Update();

      /// \brief Finalize the actor
      public: virtual void Fini();

      /// \brief update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Get the SDF values for the actor
      public: virtual const sdf::ElementPtr GetSDF();

      private: void AddSphereInertia(sdf::ElementPtr linkSdf, math::Pose pose,
                        double mass, double radius);

      private: void AddSphereCollision(sdf::ElementPtr linkSdf,
                        std::string name, math::Pose pose, double radius);

      private: void AddSphereVisual(sdf::ElementPtr linkSdf, std::string name,
                        math::Pose pose, double radius, std::string material,
                        common::Color ambient);

      private: void AddBoxVisual(sdf::ElementPtr linkSdf, std::string name,
                      math::Pose pose, math::Vector3 size, std::string material,
                      common::Color ambient);

      private: void AddActorVisual(sdf::ElementPtr linkSdf, std::string name,
                      math::Pose pose);

      protected: const common::Mesh *mesh;

      protected: common::Skeleton *skeleton;

      protected: std::string fileName;

      protected: double timeFactor;

      protected: common::Time prevSkelAnim;

      protected: common::SkeletonAnimation skelAnimation;

      protected: std::string visualName;

      protected: transport::PublisherPtr bonePosePub;
    };
    /// \}
  }
}
#endif


