/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PLANE_DEMO_PLUGIN_HH_
#define _GAZEBO_PLANE_DEMO_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class PlaneDemoPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: PlaneDemoPlugin();

    /// \brief Destructor.
    public: ~PlaneDemoPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;



    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    protected: std::string modelName;

    /// \brief Names of allowed target links, specified in sdf parameters.
    protected: std::string linkName;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    /// \brief Pointer to engine joint for applying force
    protected: physics::JointPtr engineJoint;

    /// \brief Pointer to engine joint for applying force
    protected: double throttleState;

    protected: int clIncKey;
    protected: int clDecKey;
    protected: double clIncVal;
    protected: double clDecVal;
  };
}
#endif
