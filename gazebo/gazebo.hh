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
#ifndef _GAZEBO_HH_
#define _GAZEBO_HH_

#include <gazebo/gazebo_core.hh>
#include <string>

namespace gazebo
{
  /// \brief Deprecated.
  /// \sa gazebo::printVersion.
  void print_version() GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::addPlugin.
  void add_plugin(const std::string &_filename) GAZEBO_DEPRECATED(2.3);

  /// \brief Not implemented.
  /// \sa gazebo::common::findFile.
  std::string find_file(const std::string &_file) GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::setupClient.
  /// \sa gazebo::setupServer.
  bool load(int _argc = 0, char **_argv = 0) GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::setupClient.
  /// \sa gazebo::setupServer.
  bool init() GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::setupClient.
  /// \sa gazebo::setupServer.
  void run() GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::shutdown.
  void stop() GAZEBO_DEPRECATED(2.3);

  /// \brief Deprecated.
  /// \sa gazebo::shutdown.
  void fini() GAZEBO_DEPRECATED(2.3);

  /// \brief Output version information to the terminal.
  void printVersion();

  /// \brief Add a system plugin.
  /// \param[in] _filename Path to the plugin.
  void addPlugin(const std::string &_filename);

  /// \brief Start a gazebo server. This starts transportation, and makes it
  /// possible to create worlds.
  /// \param[in] _argc Number of commandline arguments.
  /// \param[in] _argv The commandline arguments.
  /// \return True on success
  bool setupServer(int _argc = 0, char **_argv = 0);

  /// \brief Start a gazebo client. This starts transportation, and makes it
  /// possible to connect to a running simulation
  /// \param[in] _argc Number of commandline arguments.
  /// \param[in] _argv The commandline arguments.
  /// \return True on success
  bool setupClient(int _argc = 0, char **_argv = 0);

  /// \brief Create and load a new world from an SDF world file.
  /// \param[in] _worldFile The world file to load from.
  /// \return Pointer to the created world. NULL on error.
  gazebo::physics::WorldPtr loadWorld(const std::string &_worldFile);

  /// \brief Run a world for a specific number of iterations.
  /// \param[in] _world Pointer to a world.
  /// \param[in] _iterations Number of iterations to execute.
  void runWorld(gazebo::physics::WorldPtr _world, unsigned int _iterations);

  /// \brief Stop and cleanup simulation.
  /// \return True if the simulation is shutdown; false otherwise.
  bool shutdown();
}
#endif
