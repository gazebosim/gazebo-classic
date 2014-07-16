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
#ifndef _GAZEBO_DIFFDRIVE_PLUGIN_HH_
#define _GAZEBO_DIFFDRIVE_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief A simple differential drive plugin.
  class GAZEBO_VISIBLE DiffDrivePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DiffDrivePlugin();

    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf SDF parameters associated with this plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin
    public: virtual void Init();

    /// \brief On world update callback.
    private: void OnUpdate();

    /// \brief Callback that receives velocity messages.
    /// \param[in] _msg The velocity message.
    private: void OnVelMsg(ConstPosePtr &_msg);

    /// \brief Transport node.
    private: transport::NodePtr node;

    /// \brief Listens for velocity commands
    private: transport::SubscriberPtr velSub;

    /// \brief PID controlller for the left wheel
    private: common::PID leftPID;

    /// \brief PID controlller for the right wheel
    private: common::PID rightPID;

    /// \brief Model point.
    private: physics::ModelPtr model;

    /// \brief Left wheel joint
    private: physics::JointPtr leftJoint;

    /// \brief Right wheel joint
    private: physics::JointPtr rightJoint;

    /// \brief Connectes to the update world event.
    private: event::ConnectionPtr updateConnection;

    /// \brief Speed of each wheel
    private: double wheelSpeed[2];

    /// \brief The torque that can be applied to each wheel.
    private: double torque;

    /// \brief Distance between the two wheels
    private: double wheelSeparation;

    /// \brief The wheel radius for both left and right wheels
    private: double wheelRadius;

    /// \brief Force applied to left wheel
    private: double leftForce;

    /// \brief Force applied to right wheel
    private: double rightForce;

    /// \brief Previous update time
    private: common::Time prevUpdateTime;
  };
}
#endif
