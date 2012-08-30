/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef __PHYSICS_HH__
#define __PHYSICS_HH__

#include <string>

#include "physics/PhysicsTypes.hh"
#include "sdf/sdf.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    bool load();
    bool fini();

    /// \brief create a world
    WorldPtr create_world(const std::string &name ="");

    /// \brief Get back a pointer to a world by name
    WorldPtr get_world(const std::string &name = "");

    /// \brief Load world from sdf::Element pointer
    void load_world(WorldPtr world, sdf::ElementPtr _sdf);

    /// \brief initialize world
    void init_world(WorldPtr world);

    /// \brief run world by calling World::Run()
    void run_world(WorldPtr world);

    /// \brief stop world by calling World::Stop()
    void stop_world(WorldPtr world);

    /// \brief stop world by calling World::SetPaused(bool)
    void pause_world(WorldPtr world, bool pause);

    /// \brief load multiple worlds from single sdf::Element pointer
    void load_worlds(sdf::ElementPtr _sdf);

    /// \brief initialize multiple worlds stored in static variable
    ///        gazebo::g_worlds
    void init_worlds();

    /// \brief run multiple worlds stored in static variable
    ///        gazebo::g_worlds
    void run_worlds();

    /// \brief stop multiple worlds stored in static variable
    ///        gazebo::g_worlds
    void stop_worlds();

    /// \brief pause multiple worlds stored in static variable
    ///        gazebo::g_worlds
    void pause_worlds(bool pause);

    /// \brief remove multiple worlds stored in static variable
    ///        gazebo::g_worlds
    void remove_worlds();

    /// \}
  }
}
#endif


