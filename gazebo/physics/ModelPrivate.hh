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
/* Desc: Base class for all models
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */

#ifndef _MODEL_PRIVATE_HH_
#define _MODEL_PRIVATE_HH_

#include <string>
#include <map>
#include <vector>
#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/ModelState.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/util/system.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace physics
  {
    class Gripper;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Model Model.hh physics/physics.hh
    /// \brief A model is a collection of links, joints, and plugins.
    class ModelPrivate
    {
      /// used by Model::AttachStaticModel
      public: std::vector<ModelPtr> attachedModels;

      /// used by Model::AttachStaticModel
      public: std::vector<math::Pose> attachedModelsOffset;

      /// \brief Publisher for joint info.
      public: transport::PublisherPtr jointPub;

      /// \brief The canonical link of the model.
      public: LinkPtr canonicalLink;

      /// \brief All the joints in the model.
      public: Joint_V joints;

      /// \brief Cached list of links. This is here for performance.
      public: Link_V links;

      /// \brief All the grippers in the model.
      public: std::vector<GripperPtr> grippers;

      /// \brief All the model plugins.
      public: std::vector<ModelPluginPtr> plugins;

      /// \brief The joint animations.
      public: std::map<std::string, common::NumericAnimationPtr>
               jointAnimations;

      /// \brief Callback used when a joint animation completes.
      public: boost::function<void()> onJointAnimationComplete;

      /// \brief Mutex used during the update cycle.
      public: mutable boost::recursive_mutex updateMutex;

      /// \brief Controller for the joints.
      public: JointControllerPtr jointController;

    };
    /// \}
  }
}
#endif
