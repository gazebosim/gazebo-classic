/*
 * Copyright 2011 Nate Koenig
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
#ifndef _PHYSICS_HH_
#define _PHYSICS_HH_

#include <string>

#include "physics/PhysicsTypes.hh"
#include "sdf/sdf.hh"

namespace gazebo
{
  /// \ingroup gazebo_physics
  /// \brief physics namespace
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Setup gazebo::SystemPlugin's and call gazebo::transport::init.
    bool load();

    /// \brief Finalize transport by calling gazebo::transport::fini.
    bool fini();

    /// \brief Create a world given a name.
    WorldPtr create_world(const std::string &name ="");

    /// \brief Returns a pointer to a world by name
    WorldPtr get_world(const std::string &name = "");

    /// \brief Load world from sdf::Element pointer
    void load_world(WorldPtr world, sdf::ElementPtr _sdf);

    /// \brief Init world given a pointer to it.
    void init_world(WorldPtr world);

    /// \brief Run world by calling World::Run() given a pointer to it.
    void run_world(WorldPtr world);

    /// \brief Stop world by calling World::Stop() given a pointer to it.
    void stop_world(WorldPtr world);

    /// \brief Pause world by calling World::SetPaused.
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


