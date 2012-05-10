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
#ifndef __JOINTCONTROLLER_HH__
#define __JOINTCONTROLLER_HH__

#include <map>
#include <string>
#include <vector>

#include "common/PID.hh"
#include "common/Time.hh"
#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"
#include "msgs/msgs.h"

namespace gazebo
{
  namespace physics
  {
    class JointController
    {
      public: JointController(ModelPtr _model);
      public: void AddJoint(JointPtr _joint);
      public: void Update();

      /// \brief Set the position of a joint
      public: void SetJointPosition(const std::string &_name, double _position);

      /// \brief Set the positions of a set of joints
      public: void SetJointPositions(
                  const std::map<std::string, double> &_jointPositions);

      private: void OnJointCmd(ConstJointCmdPtr &_msg);

      private: void SetJointPosition(JointPtr _joint, double _position);

      /// \brief Helper for SetJointPositions
      private: void RotateBodyAndChildren(LinkPtr _link1,
                   const math::Vector3 &_anchor, const math::Vector3 &_axis,
                   double _dangle, bool _updateChildren);

      /// \brief Helper for SetJointPositions
      private: void SlideBodyAndChildren(LinkPtr _link1,
                   const math::Vector3 &_anchor, const math::Vector3 &_axis,
                   double _dposition, bool _updateChildren);

      /// \brief Helper for SetJointPositions
      private: void GetAllChildrenBodies(std::vector<LinkPtr> &_bodies,
                                         const LinkPtr &_body);

      /// \brief Helper for SetJointPositions
      private: void GetAllParentBodies(std::vector<LinkPtr> &_bodies,
                   const LinkPtr &_body, const LinkPtr &_origParentBody);

      /// \brief Helper for SetJointPositions
      private: bool InBodies(const LinkPtr &_body,
                             const std::vector<LinkPtr> &_bodies);

      private: ModelPtr model;
      private: std::map<std::string, JointPtr> joints;
      private: std::map<std::string, common::PID> pids;

      private: std::map<std::string, double> forces;
      private: std::map<std::string, double> positions;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr jointCmdSub;

      private: common::Time prevUpdateTime;
    };
  }
}
#endif
