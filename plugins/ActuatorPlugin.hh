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

#ifndef _GAZEBO_ACTUATOR_PLUGIN_
#define _GAZEBO_ACTUATOR_PLUGIN_

#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  /// \brief Properties for a model of a rotational actuator
  class ActuatorProperties
  {
    /// \brief An identifier for the actuator.
    public: std::string name;

    /// \brief Which joint index is actuated by this actuator.
    public: int jointIndex;

    /// \brief Mechanical power output of the actuator (Watts)
    public: float power;

    /// \brief Maximum velocity of the actuator (radians per second)
    public: float maximumVelocity;

    /// \brief Maximum torque of the actuator (Newton-meters)
    public: float maximumTorque;

    public: boost::function<float (float, float, const ActuatorProperties&)>
              modelFunction;
  };

  class ActuatorPlugin : public ModelPlugin
  {
    /// \brief Load the plugin from SDF.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief The joints we want to actuate
    private: std::vector<physics::JointPtr> joints;

    /// \brief Corresponding actuator properties (power, max torque, etc.)
    private: std::vector<ActuatorProperties> actuators;

    /// \brief Connections to events associated with this class.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}

#endif
