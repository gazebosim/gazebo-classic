/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_SASCPLUGIN_HH_
#define GAZEBO_PLUGINS_SASCPLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  // Forward declare private data class.
  class SASCPluginPrivate;

  class GAZEBO_VISIBLE SASCPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: SASCPlugin();

    /// \brief Destructor.
    public: ~SASCPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    public: virtual void Init();

    /// \brief Add a new model to the queue of drones that will be launched
    /// \param[in] _model Pointer to the model
    public: void AddModel(physics::ModelPtr _model);

    /// \brief Callback when a new model msg is received
    /// \param[in] _msg Message describing the new model
    public: void OnModel(ConstModelPtr &_msg);

    /// \brief Process model messages and check for fixed wing drones
    public: void ProcessModelMsgs();

    /// \brief Main physics update loop
    private: void OnUpdate();

    private: std::unique_ptr<SASCPluginPrivate> dataPtr;
  };
}
#endif
