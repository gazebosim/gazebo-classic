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
#ifndef GAZEBO_PHYSICS_MODELPRIVATE_HH_
#define GAZEBO_PHYSICS_MODELPRIVATE_HH_

#include <vector>
#include <mutex>
#include <map>
#include <string>
#include <ignition/math/Pose3.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/EntityPrivate.hh"


namespace gazebo
{
  namespace physics
  {
    /// \brief Private Model data
    class ModelPrivate : public EntityPrivate
    {
      /// used by Model::AttachStaticModel
      public: std::vector<ModelPtr> attachedModels;

      /// used by Model::AttachStaticModel
      public: std::vector<ignition::math::Pose3d> attachedModelsOffset;

      /// \brief Publisher for joint info.
      public: transport::PublisherPtr jointPub;

      /// \brief The canonical link of the model.
      public: LinkPtr canonicalLink;

      /// \brief All the joints in the model.
      public: Joint_V joints;

      /// \brief Cached list of links. This is here for performance.
      public: Link_V links;

      /// \brief Cached list of nested models.
      public: Model_V models;

      /// \brief All the grippers in the model.
      public: std::vector<GripperPtr> grippers;

      /// \brief All the model plugins.
      public: std::vector<ModelPluginPtr> plugins;

      /// \brief The joint animations.
      public: std::map<std::string, common::NumericAnimationPtr>
               jointAnimations;

      /// \brief Callback used when a joint animation completes.
      public: std::function<void()> onJointAnimationComplete;

      /// \brief Mutex used during the update cycle.
      public: mutable std::recursive_mutex updateMutex;

      /// \brief Controller for the joints.
      public: JointControllerPtr jointController;
    };
  }
}
#endif
