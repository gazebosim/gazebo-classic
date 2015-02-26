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
#ifndef _GAZEBO_JOINTCONTROLLER_HH_
#define _GAZEBO_JOINTCONTROLLER_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data values.
    class JointControllerPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class JointController JointController.hh physics/physics.hh
    /// \brief A class for manipulating physics::Joint
    class GAZEBO_VISIBLE JointController
    {
      /// \brief Constructor
      /// \param[in] _model Model that uses this joint controller.
      public: explicit JointController(ModelPtr _model);

      /// \brief Destructor
      public: virtual ~JointController();

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

      /// \brief Get the last time the controller was updated.
      /// \return Last time the controller was updated.
      public: common::Time GetLastUpdateTime() const;

      /// \brief Get all the joints.
      /// \return A map<joint_name, joint_ptr> to all the joints that can
      /// be controlled.
      public: std::map<std::string, JointPtr> GetJoints() const;

      /// \brief Set the position PID values for a joint.
      /// \param[in] _jointName Scoped name of the joint.
      /// \param[in] _pid New position PID controller.
      public: void SetPositionPID(const std::string &_jointName,
                  const common::PID &_pid);

      /// \brief Set the target position for the position PID controller.
      /// \param[in] _jointName Scoped name of the joint.
      /// \param[in] _target Position target.
      /// \return False if the joint was not found.
      public: bool SetPositionTarget(const std::string &_jointName,
                  double _target);

      /// \brief Set the velocity PID values for a joint.
      /// \param[in] _jointName Scoped name of the joint.
      /// \param[in] _pid New velocity PID controller.
      public: void SetVelocityPID(const std::string &_jointName,
                  const common::PID &_pid);

      /// \brief Set the target velocity for the velocity PID controller.
      /// \param[in] _jointName Scoped name of the joint.
      /// \param[in] _target Velocity target.
      /// \return False if the joint was not found.
      public: bool SetVelocityTarget(const std::string &_jointName,
                  double _target);

      /// \brief Get all the position PID controllers.
      /// \return A map<joint_name, PID> for all the position PID
      /// controllers.
      public: std::map<std::string, common::PID> GetPositionPIDs() const;

      /// \brief Get all the velocity PID controllers.
      /// \return A map<joint_name, PID> for all the velocity PID
      /// controllers.
      public: std::map<std::string, common::PID> GetVelocityPIDs() const;

      /// \brief Get all the applied forces.
      /// \return A map<joint_name, force> that contains force values set by
      /// the user of the JointController.
      public: std::map<std::string, double> GetForces() const;

      /// \brief Get all the position PID set points.
      /// \return A map<joint_name, position> that contains position values
      /// set by the user of the JointController.
      public: std::map<std::string, double> GetPositions() const;

      /// \brief Get all the velocity PID set points.
      /// \return A map<joint_name, position> that contains velocity values
      /// set by the user of the JointController.
      public: std::map<std::string, double> GetVelocities() const;

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
      /// Warning:
      /// \param[in] _joint Joint to set.
      /// \param[in] _position Position of the joint.
      public: void SetJointPosition(
        JointPtr _joint, double _position, int _index = 0);

      /// \brief Private data values.
      private: JointControllerPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
