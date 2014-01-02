/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#ifndef _MOTOR_DEMO_PLUGIN_HH_
#define _MOTOR_DEMO_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class MotorDemoPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: MotorDemoPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    private: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    private: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    private: physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    private: std::string modelName;

    /// \brief max torque of the motor.
    private: double maxTorque;

    /// \brief torque speed slope of the motor.
    private: double torqueSpeedSlope;

    /// \brief motor shaft joint name.
    /// parent link is the motor body.
    /// child link is the spinning shaft.
    private: std::string motorShaftJointName;

    /// \brief pointer to motor shaft joint.
    private: physics::JointPtr motorShaftJoint;

    /// \brief motor position/velocity encoder joint name.
    /// Joint position is used as encoder sensor output.
    private: std::string encoderJointName;

    /// \brief pointer to encoder joint.
    private: physics::JointPtr encoderJoint;

    /// \brief motor force torque sensor joint name.
    /// force and torque is used to construct ft sensor output.
    private: std::string forceTorqueSensorJointName;

    /// \brief pointer to force torque sensor joint.
    private: physics::JointPtr forceTorqueSensorJoint;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;
  };
}
#endif  // ifndef _MOTOR_DEMO_PLUGIN_HH_
