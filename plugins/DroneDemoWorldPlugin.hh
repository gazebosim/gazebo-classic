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

#ifndef _GAZEBO_DRONE_DEMO_WORLD_PLUGIN_HH_
#define _GAZEBO_DRONE_DEMO_WORLD_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    class Joystick;
  }

  class GAZEBO_VISIBLE DroneDemoWorldPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: DroneDemoWorldPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin
    public: virtual void Reset();

    /// \brief Callback that updates the timer and reset the world if needed.
    private: virtual void OnUpdate();

    private: physics::WorldPtr world;

    private: common::Timer timer;

    private: common::Time timeLimit;

    private: event::ConnectionPtr updateConnection;

    private: transport::NodePtr node;

    private: transport::PublisherPtr worldControlPub;


    /// \brief Pointer to the joystick interface.
    private: util::Joystick *joy;
  };
}
#endif
