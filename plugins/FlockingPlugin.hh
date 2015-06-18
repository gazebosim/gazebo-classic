/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_FLOCKING_PLUGIN_HH_
#define _GAZEBO_FLOCKING_PLUGIN_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief Implements a flocking behavior.
  class GAZEBO_VISIBLE FlockingPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: FlockingPlugin();

    /// \brief Destructor.
    public: virtual ~FlockingPlugin() = default;

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Called every update cycle
    private: void OnUpdate();

    /// Pointer to the model.
    private: physics::ModelPtr model;

    /// Pointer to the world.
    private: physics::WorldPtr world;

    /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: double linearSpeed;
    private: double angularSpeed;
  };
}
#endif
