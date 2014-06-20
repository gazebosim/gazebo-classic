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
#ifndef _JOINTCONTROLLER_HH_
#define _JOINTCONTROLLER_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class JointController JointController.hh physics/physics.hh
    /// \brief A class for manipulating physics::Joint
    class JointController
    {
      /// \brief Constructor
      /// \param[in] _model Model that uses this joint controller.
      public: explicit JointController(ModelPtr _model);

      /// \brief Add a joint to control.
      /// \param[in] _joint Joint to control.
      public: void AddJoint(JointPtr _joint);

      /// \brief Update the joint control.
      public: void Update();

      /// \brief Reset all commands
      public: void Reset();

      /// \brief Set the positions of a Joint by name.
      /// \sa JointController::SetJointPosition(JointPtr, double)
      public: void SetJointPosition(
        const std::string &_name, double _position, int _index = 0);

      /// \brief Set the positions of a set of Joint's.
      /// \sa JointController::SetJointPosition(JointPtr, double)
      public: void SetJointPositions(
                  const std::map<std::string, double> &_jointPositions);

      /// \brief Callback when a joint command message is received.
      /// \param[in] _msg The received message.
      private: void OnJointCmd(ConstJointCmdPtr &_msg);

      /// \brief Set the positions of a Joint by name
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
      /// \param[in] _joint Joint to set.
      /// \param[in] _position Position of the joint.
      public: void SetJointPosition(
        JointPtr _joint, double _position, int _index = 0);

      /// \brief Helper for SetJointPositions.
      /// \param[in] _joint Joint to move.
      /// \param[in] _link Link to move.
      /// \param[in] _anchor Anchor position of the joint.
      /// \param[in] _axis Axis of the joint.
      /// \param[in] _dposition Rotation angle.
      /// \param[in] _updateChildren Update child joints.
      private: void MoveLinks(JointPtr _joint, LinkPtr _link,
                   const math::Vector3 &_anchor, const math::Vector3 &_axis,
                   double _dposition, bool _updateChildren = false);

      /// \internal
      /// \TODO: Set Link Velocity based on old and new poses and dt
      private: void ComputeAndSetLinkTwist(LinkPtr _link,
                    const math::Pose &_old, const math::Pose &_new, double dt);

      /// \brief Helper for SetJointPositions
      /// \param[out] _linksOut All the connected links.
      /// \param[in] _link Link to get the connections from.
      /// \param[in] _checkParentTree True to recurse down parent link trees.
      private: void AddConnectedLinks(std::vector<LinkPtr> &_linksOut,
                                      const LinkPtr &_link,
                                      bool _checkParentTree = false);

      /// \brief Helper for SetJointPositions.
      /// \param[in] _vector List of links.
      /// \param[in] _value Link to check against.
      private: template<class InputVector, class T>
                 bool ContainsLink(InputVector _vector, const T &_value)
                 {
                   typename InputVector::iterator iter = _vector.begin();
                   for (; iter != _vector.end(); ++iter)
                     if ((*iter)->GetScopedName() == _value->GetScopedName())
                       return true;
                   return false;
                 }

      /// \brief Model to control.
      private: ModelPtr model;

      /// \brief List of links that have been updated.
      private: Link_V updatedLinks;

      /// \brief Map of joint names to the joint pointer.
      private: std::map<std::string, JointPtr> joints;

      /// \brief Position PID controllers.
      private: std::map<std::string, common::PID> posPids;

      /// \brief Velocity PID controllers.
      private: std::map<std::string, common::PID> velPids;

      /// \brief Forces applied to joints.
      private: std::map<std::string, double> forces;

      /// \brief Joint positions.
      private: std::map<std::string, double> positions;

      /// \brief Joint velocities.
      private: std::map<std::string, double> velocities;

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// \brief Subscribe to joint command.
      private: transport::SubscriberPtr jointCmdSub;

      /// \brief Last time the controller was updated.
      private: common::Time prevUpdateTime;
    };
    /// \}
  }
}
#endif
