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
#ifndef _LIFT_DRAG_PLUGIN_HH_
#define _LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class LiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LiftDragPlugin();

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

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    private: double cla;

    /// \brief Coefficient of Drag / alpha slope.
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    private: double cda;

    /// \brief Coefficient of Moment / alpha slope.
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    private: double cma;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    private: double rho;

    /// \brief effective planeform surface area
    private: double area;

    /// \brief angle of sweep
    private: double sweep;

    /// \brief initial angle of attack
    private: double a0;

    /// \brief angle of attack
    private: double alpha;

    /// \brief center of pressure in link local coordinates
    private: math::Vector3 cp;

    /// \brief forward flight direction in link local coordinates
    private: math::Vector3 forward;

    /// \brief A vector in the lift/drag plane, anything orthogonal to it
    /// is considered wing sweep.
    private: math::Vector3 upward;

    /// \brief Smooth velocity
    private: math::Vector3 velSmooth;

    /// \brief Names of allowed target links, specified in sdf parameters.
    private: std::string linkName;

    /// \brief Pointer to link currently targeted by mud joint.
    private: physics::LinkPtr link;

    /// \brief SDF for this plugin;
    private: sdf::ElementPtr sdf;
  };
}
#endif  // ifndef _LIFT_DRAG_PLUGIN_HH_
