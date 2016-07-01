/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_HH_
#define _GAZEBO_HH_

#include <gazebo/gazebo_core.hh>
#include <string>
#include <vector>
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief Output version information to the terminal.
  GAZEBO_VISIBLE
  void printVersion();

  /// \brief Add a system plugin.
  /// \param[in] _filename Path to the plugin.
  GAZEBO_VISIBLE
  void addPlugin(const std::string &_filename);

  /// \brief Start a gazebo server. This starts transportation, and makes it
  /// possible to create worlds.
  /// \param[in] _argc Number of commandline arguments.
  /// \param[in] _argv The commandline arguments.
  /// \return True on success.
  GAZEBO_VISIBLE
  bool setupServer(int _argc = 0, char **_argv = 0);

  /// \brief Start a gazebo server. This starts transportation, and makes it
  /// possible to create worlds.
  /// \param[in] _args Vector of arguments only parsed by the system plugins.
  /// Note that when you run gazebo/gzserver, all the options (--version,
  /// --server-plugin, etc.) are parsed but when using Gazebo as a library, the
  /// arguments are only parsed by the system plugins.
  /// \sa gazebo::SystemPlugin::Load()
  /// \return True on success.
  GAZEBO_VISIBLE
  bool setupServer(const std::vector<std::string> &_args);

  /// \brief Create and load a new world from an SDF world file.
  /// \param[in] _worldFile The world file to load from.
  /// \return Pointer to the created world. NULL on error.
  GAZEBO_VISIBLE
  gazebo::physics::WorldPtr loadWorld(const std::string &_worldFile);

  /// \brief Run a world for a specific number of iterations.
  /// \param[in] _world Pointer to a world.
  /// \param[in] _iterations Number of iterations to execute.
  GAZEBO_VISIBLE
  void runWorld(gazebo::physics::WorldPtr _world, unsigned int _iterations);

  /// \brief Stop and cleanup simulation.
  /// \return True if the simulation is shutdown; false otherwise.
  GAZEBO_VISIBLE
  bool shutdown();
}

#endif
