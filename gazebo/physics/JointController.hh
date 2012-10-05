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
#include "msgs/msgs.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A class for manipulating physics::Joint
    class JointController
    {
      public: JointController(ModelPtr _model);
      public: void AddJoint(JointPtr _joint);
      public: void Update();

      /// \brief Reset all commands
      public: void Reset();

      /// Set the positions of a Joint by name
      ///   \sa JointController::SetJointPosition(JointPtr, double)
      public: void SetJointPosition(const std::string &_name, double _position);

      /// Set the positions of a set of Joint's.
      ///   \sa JointController::SetJointPosition(JointPtr, double)
      public: void SetJointPositions(
                  const std::map<std::string, double> &_jointPositions);

      private: void OnJointCmd(ConstJointCmdPtr &_msg);

      /// Set the positions of a Joint by name
      ///        The position is specified in native units, which means,
      ///        if you are using metric system, it's meters for SliderJoint
      ///        and radians for HingeJoint, etc.
      /// Implementation:
      ///   In order to change the position of a Joint inside a Model,
      ///   this call must recursively crawl through all the connected
      ///   children Link's in this Model, and update each Link Pose
      ///   affected by this Joint angle update.
      /// Warning:
      ///   There is no constraint satisfaction being done here,
      ///   traversal through the kinematic graph has unexpected behavior
      ///   if you try to set the joint position of a link inside
      ///   a loop structure.
      public: void SetJointPosition(JointPtr _joint, double _position);

      /// \brief Helper for SetJointPositions
      private: void MoveLinks(JointPtr _joint, LinkPtr _link,
                   const math::Vector3 &_anchor, const math::Vector3 &_axis,
                   double _dposition, bool _updateChildren = false);

      /// @todo: Set Link Velocity based on old and new poses and dt
      private: void ComputeAndSetLinkTwist(LinkPtr _link,
                    const math::Pose &_old, const math::Pose &_new, double dt);

      /// \brief Helper for SetJointPositions
      private: void AddConnectedLinks(std::vector<LinkPtr> &_links_out,
                                      const LinkPtr &_link,
                                      bool _checkParentTree = false);

      /// \brief Helper for SetJointPositions
      private: template<class InputVector, class T>
                 bool ContainsLink(InputVector _vector, const T& value)
                 {
                   typename InputVector::iterator iter = _vector.begin();
                   for (; iter != _vector.end(); ++iter)
                     if ((*iter)->GetScopedName() == value->GetScopedName())
                       return true;
                   return false;
                 }

      private: ModelPtr model;
      private: std::vector<LinkPtr> updated_links;
      private: std::map<std::string, JointPtr> joints;
      private: std::map<std::string, common::PID> posPids;
      private: std::map<std::string, common::PID> velPids;

      private: std::map<std::string, double> forces;
      private: std::map<std::string, double> positions;
      private: std::map<std::string, double> velocities;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr jointCmdSub;

      private: common::Time prevUpdateTime;
    };
    /// \}
  }
}
#endif
