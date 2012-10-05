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
#ifndef _GRIPPER_HH_
#define _GRIPPER_HH_

#include <map>
#include <vector>
#include <string>

#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A gripper abstraction
    /*
     * A gripper is a collection of links that act as a gripper. This class
     * will intelligently generate fixed joints between the gripper and an
     * object within the gripper. This allows the object to be manipulated
     * without falling or behaving poorly.
     */
    class Gripper
    {
      /// \brief Constructor
      public: Gripper(ModelPtr _model);

      /// \brief Destructor
      public: virtual ~Gripper();

      /// \brief Load
      /// \param _sdf Shared point to an sdf element that contains the list
      /// of links in the gripper.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize
      public: virtual void Init();

      /// \brief Update the gripper
      private: void OnUpdate();

      /// \brief Callback used when the gripper contacts an object
      private: void OnContact(const std::string &_collisionName,
                              const physics::Contact &_contact);

      /// \brief Attach an object to the gripper
      private: void HandleAttach();

      /// \brief Detach an object from the gripper
      private: void HandleDetach();

      /// \brief A reset function
      private: void ResetDiffs();

      private: physics::ModelPtr model;
      private: physics::PhysicsEnginePtr physics;
      private: physics::JointPtr fixedJoint;

      private: physics::LinkPtr palmLink;
      private: std::vector<event::ConnectionPtr> connections;

      private: std::map<std::string, physics::CollisionPtr> collisions;
      private: std::vector<physics::Contact> contacts;

      private: bool attached;

      private: math::Pose prevDiff;
      private: std::vector<double> diffs;
      private: int diffIndex;

      private: common::Time updateRate, prevUpdateTime;
      private: int posCount, zeroCount;
      /// \brief minimum number of links touching
      private: unsigned int min_contact_count;
      /// \brief Steps touching before engaging fixed joint
      private: int attach_steps;
      /// \brief Steps not touching before deisengaging fixed joint
      private: int detach_steps;
    };
    /// \}
  }
}
#endif
