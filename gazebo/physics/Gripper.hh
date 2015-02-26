/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Gripper Gripper.hh physics/physics.hh
    /// \brief A gripper abstraction
    ///
    /// A gripper is a collection of links that act as a gripper. This class
    /// will intelligently generate fixed joints between the gripper and an
    /// object within the gripper. This allows the object to be manipulated
    /// without falling or behaving poorly.
    class GAZEBO_VISIBLE Gripper
    {
      /// \brief Constructor
      /// \param[in] _model The model which contains the Gripper.
      public: explicit Gripper(ModelPtr _model);

      /// \brief Destructor.
      public: virtual ~Gripper();

      /// \brief Load the gripper.
      /// \param[in] _sdf Shared point to an sdf element that contains the list
      /// of links in the gripper.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize.
      public: virtual void Init();

      /// \brief Return the name of the gripper.
      public: std::string GetName() const;

      /// \brief True if the gripper is attached to another model.
      /// \return True if the gripper is active and a joint has been
      /// created between the gripper and another model.
      public: bool IsAttached() const;

      /// \brief Update the gripper.
      private: void OnUpdate();

      /// \brief Callback used when the gripper contacts an object.
      /// \param[in] _msg Message that contains contact information.
      private: void OnContacts(ConstContactsPtr &_msg);

      /// \brief Attach an object to the gripper.
      private: void HandleAttach();

      /// \brief Detach an object from the gripper.
      private: void HandleDetach();

      /// \brief A reset function.
      private: void ResetDiffs();

      /// \brief Model that contains this gripper.
      private: physics::ModelPtr model;

      /// \brief The physics engine.
      private: physics::PhysicsEnginePtr physics;

      /// \brief Pointer to the world.
      private: physics::WorldPtr world;

      /// \brief A fixed joint to connect the gripper to a grasped object.
      private: physics::JointPtr fixedJoint;

      /// \brief The base link for the gripper.
      private: physics::LinkPtr palmLink;

      /// \brief All our connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief The collisions for the links in the gripper.
      private: std::map<std::string, physics::CollisionPtr> collisions;

      /// \brief The current contacts.
      private: std::vector<msgs::Contact> contacts;

      /// \brief Mutex used to protect reading/writing the sonar message.
      private: boost::mutex mutexContacts;

      /// \brief True if the gripper has an object.
      private: bool attached;

      /// \brief Previous difference between the palm link and grasped
      /// object.
      private: math::Pose prevDiff;

      /// \brief Used to determine when to create the fixed joint.
      private: std::vector<double> diffs;

      /// \brief Current index into the diff array.
      private: int diffIndex;

      /// \brief Rate at which to update the gripper.
      private: common::Time updateRate;

      /// \brief Previous time when the gripper was updated.
      private: common::Time prevUpdateTime;

      /// \brief Number of iterations the gripper was contacting the same
      /// object.
      private: int posCount;

      /// \brief Number of iterations the gripper was not contacting the same
      /// object.
      private: int zeroCount;

      /// \brief Minimum number of links touching.
      private: unsigned int minContactCount;

      /// \brief Steps touching before engaging fixed joint
      private: int attachSteps;

      /// \brief Steps not touching before disengaging fixed joint
      private: int detachSteps;

      /// \brief Name of the gripper.
      private: std::string name;

      /// \brief Node for communication.
      protected: transport::NodePtr node;

      /// \brief Subscription to contact messages from the physics engine.
      private: transport::SubscriberPtr contactSub;
    };
    /// \}
  }
}
#endif
