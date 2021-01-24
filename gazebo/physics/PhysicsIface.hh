/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/msgs/poses_stamped.pb.h"

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
    GZ_PHYSICS_VISIBLE
    bool load();

    /// \brief Finalize transport by calling gazebo::transport::fini.
    GZ_PHYSICS_VISIBLE
    bool fini();

    /// \brief Create a world given a name.
    /// \param[in] _name Name of the world to create.
    /// \return Pointer to the new world.
    GZ_PHYSICS_VISIBLE
    WorldPtr create_world(const std::string &_name ="");

    /// \brief Returns a pointer to a world by name.
    /// \param[in] _name Name of the world to get.
    /// \return Pointer to the world.
    GZ_PHYSICS_VISIBLE
    WorldPtr get_world(const std::string &_name = "");

    /// \brief checks if the world with this name exists.
    /// Can be used to check if get_world(const std::string&)
    /// will succeed or throw an exception.
    /// \param[in] _name Name of the world to check for, or
    ///   empty to check if *any* world has been loaded.
    /// \return true if the world exists.
    GZ_PHYSICS_VISIBLE
    bool has_world(const std::string &_name = "");

    /// \brief Load world from sdf::Element pointer.
    /// \param[in] _world Pointer to a world.
    /// \param[in] _sdf SDF values to load from.
    GZ_PHYSICS_VISIBLE
    void load_world(WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Init world given a pointer to it.
    /// \param[in] _world World to initialize.
    GZ_PHYSICS_VISIBLE
    void init_world(WorldPtr _world);

    /// \brief Init world given a pointer to it.
    /// \param[in] _world World to initialize.
    /// \param[in] _func function to be called when Poses are available.
    GZ_PHYSICS_VISIBLE
    void init_world(WorldPtr _world, std::function<
        void(const std::string &, const msgs::PosesStamped &)> _func);

    /// \brief Run world by calling World::Run() given a pointer to it.
    /// \param[in] _world World to run.
    /// \param[in] _iterations Number of iterations for each world to take.
    /// Zero indicates that each world should continue forever.
    GZ_PHYSICS_VISIBLE
    void run_world(WorldPtr _world, unsigned int _iterations = 0);

    /// \brief Stop world by calling World::Stop() given a pointer to it.
    /// \param[in] _world World to stop.
    GZ_PHYSICS_VISIBLE
    void stop_world(WorldPtr _world);

    /// \brief Pause world by calling World::SetPaused.
    /// \param[in] _world World to pause or unpause.
    /// \param[in] _pause True to pause, False to unpause.
    GZ_PHYSICS_VISIBLE
    void pause_world(WorldPtr _world, bool _pause);

    /// \brief load multiple worlds from single sdf::Element pointer
    /// \param[in] _sdf SDF values used to create worlds.
    GZ_PHYSICS_VISIBLE
    void load_worlds(sdf::ElementPtr _sdf);

    /// \brief initialize multiple worlds stored in static variable
    /// gazebo::g_worlds
    GZ_PHYSICS_VISIBLE
    void init_worlds();

    /// \brief initialize multiple worlds stored in static variable
    /// gazebo::g_worlds
    /// \param[in] _func function to be called when Poses are available.
    GZ_PHYSICS_VISIBLE
    void init_worlds(std::function<
        void(const std::string &, const msgs::PosesStamped &)> _func);

    /// \brief Run multiple worlds stored in static variable
    /// gazebo::g_worlds
    /// \param[in] _iterations Number of iterations for each world to take.
    /// Zero indicates that each world should continue forever.
    GZ_PHYSICS_VISIBLE
    void run_worlds(unsigned int _iterations = 0);

    /// \brief stop multiple worlds stored in static variable
    /// gazebo::g_worlds
    GZ_PHYSICS_VISIBLE
    void stop_worlds();

    /// \brief pause multiple worlds stored in static variable
    /// gazebo::g_worlds
    /// \param[in] _pause True to pause, False to unpause.
    GZ_PHYSICS_VISIBLE
    void pause_worlds(bool pause);

    /// \brief remove multiple worlds stored in static variable
    /// gazebo::g_worlds
    GZ_PHYSICS_VISIBLE
    void remove_worlds();

    /// \brief Return true if any world is running.
    /// \return True if any world is running.
    GZ_PHYSICS_VISIBLE
    bool worlds_running();

    /// \brief Get a unique ID
    /// \return A unique integer
    GZ_PHYSICS_VISIBLE
    uint32_t getUniqueId();
    /// \}
  }
}
#endif
