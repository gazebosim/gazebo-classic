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
#ifndef _PHYSICSIFACE_HH_
#define _PHYSICSIFACE_HH_

#include <string>
#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_physics
  /// \brief physics namespace
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Setup gazebo::SystemPlugin's and call gazebo::transport::init.
    GAZEBO_VISIBLE
    bool load();

    /// \brief Finalize transport by calling gazebo::transport::fini.
    GAZEBO_VISIBLE
    bool fini();

    /// \brief Create a world given a name.
    /// \param[in] _name Name of the world to create.
    /// \return Pointer to the new world.
    GAZEBO_VISIBLE
    WorldPtr create_world(const std::string &_name ="");

    /// \brief Returns a pointer to a world by name.
    /// \param[in] _name Name of the world to get.
    /// \return Pointer to the world.
    GAZEBO_VISIBLE
    WorldPtr get_world(const std::string &_name = "");

    /// \brief Load world from sdf::Element pointer.
    /// \param[in] _world Pointer to a world.
    /// \param[in] _sdf SDF values to load from.
    GAZEBO_VISIBLE
    void load_world(WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Init world given a pointer to it.
    /// \param[in] _world World to initialize.
    GAZEBO_VISIBLE
    void init_world(WorldPtr _world);

    /// \brief Run world by calling World::Run() given a pointer to it.
    /// \param[in] _world World to run.
    /// \param[in] _iterations Number of iterations for each world to take.
    /// Zero indicates that each world should continue forever.
    GAZEBO_VISIBLE
    void run_world(WorldPtr _world, unsigned int _iterations = 0);

    /// \brief Stop world by calling World::Stop() given a pointer to it.
    /// \param[in] _world World to stop.
    GAZEBO_VISIBLE
    void stop_world(WorldPtr _world);

    /// \brief Pause world by calling World::SetPaused.
    /// \param[in] _world World to pause or unpause.
    /// \param[in] _pause True to pause, False to unpause.
    GAZEBO_VISIBLE
    void pause_world(WorldPtr _world, bool _pause);

    /// \brief load multiple worlds from single sdf::Element pointer
    /// \param[in] _sdf SDF values used to create worlds.
    GAZEBO_VISIBLE
    void load_worlds(sdf::ElementPtr _sdf);

    /// \brief initialize multiple worlds stored in static variable
    /// gazebo::g_worlds
    GAZEBO_VISIBLE
    void init_worlds();

    /// \brief Run multiple worlds stored in static variable
    /// gazebo::g_worlds
    /// \param[in] _iterations Number of iterations for each world to take.
    /// Zero indicates that each world should continue forever.
    GAZEBO_VISIBLE
    void run_worlds(unsigned int _iterations = 0);

    /// \brief stop multiple worlds stored in static variable
    /// gazebo::g_worlds
    GAZEBO_VISIBLE
    void stop_worlds();

    /// \brief pause multiple worlds stored in static variable
    /// gazebo::g_worlds
    /// \param[in] _pause True to pause, False to unpause.
    GAZEBO_VISIBLE
    void pause_worlds(bool pause);

    /// \brief remove multiple worlds stored in static variable
    /// gazebo::g_worlds
    GAZEBO_VISIBLE
    void remove_worlds();

    /// \brief Return true if any world is running.
    /// \return True if any world is running.
    GAZEBO_VISIBLE
    bool worlds_running();

    /// \brief Get a unique ID
    /// \return A unique integer
    GAZEBO_VISIBLE
    uint32_t getUniqueId();
    /// \}
  }
}
#endif
