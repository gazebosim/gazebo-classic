/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_JOINTCONTROLLER_PRIVATE_HH_
#define _GAZEBO_JOINTCONTROLLER_PRIVATE_HH_

#include <string>
#include <map>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class JointControllerPrivate
    {
      /// \brief Model to control.
      public: ModelPtr model;

      /// \brief List of links that have been updated.
      public: Link_V updatedLinks;

      /// \brief Map of joint names to the joint pointer.
      public: std::map<std::string, JointPtr> joints;

      /// \brief Position PID controllers.
      public: std::map<std::string, common::PID> posPids;

      /// \brief Velocity PID controllers.
      public: std::map<std::string, common::PID> velPids;

      /// \brief Forces applied to joints.
      public: std::map<std::string, double> forces;

      /// \brief Joint positions.
      public: std::map<std::string, double> positions;

      /// \brief Joint velocities.
      public: std::map<std::string, double> velocities;

      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Subscribe to joint command.
      public: transport::SubscriberPtr jointCmdSub;

      /// \brief Last time the controller was updated.
      public: common::Time prevUpdateTime;
    };
  }
}
#endif
