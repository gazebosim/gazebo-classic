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

    /// \brief Stiffness parameter, used in conjunction with damping
    ///        to compute joint erp, cfm
    private: double stiffness;

    /// \brief Damping parameter, used in conjunction with stiffness
    ///        to compute joint erp, cfm
    private: double damping;

    /// \brief Names of allowed target links, specified in sdf parameters.
    private: std::vector<std::string> linkNames;

    /// \brief Pointer to link currently targeted by mud joint.
    private: std::vector<physics::LinkPtr> links;

    /// \brief Dynamically created joint for simulating mud forces.
    private: std::vector<physics::JointPtr> joints;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;
  };
}
#endif  // ifndef _MOTOR_DEMO_PLUGIN_HH_
