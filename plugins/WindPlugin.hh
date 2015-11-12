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

#ifndef _GAZEBO_WIND_PLUGIN_HH_
#define _GAZEBO_WIND_PLUGIN_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  /// \brief A plugin that simulates a sine wind.
  class GAZEBO_VISIBLE WindPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: WindPlugin();

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Get the global wind velocity.
    /// \param[in] _wind Reference to the wind.
    /// \param[in] _wind Pointer to an entity at which location the wind
    /// velocity is to be calculated.
    /// \return Wind's velocity at entity's location.
    public: ignition::math::Vector3d LinearVel(const physics::WindPtr &_wind,
                                               const physics::Entity *_entity);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
