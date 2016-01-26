/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SHARED_INTERFACE_HH_
#define _GAZEBO_SHARED_INTERFACE_HH_

/// \internal
#include <string>
#include <vector>
#include "gazebo/common/CommonTypes.hh"

namespace gazebo_shared
{
  /// \internal
  /// \brief Output version information to the terminal.
  void printVersion();

  /// \internal
  /// \brief Add a system plugin.
  /// \param[in] _filename Path to the plugin.
  /// \param[in] _plugins Vector of plugins into which the new plugin should
  /// be added
  void addPlugin(const std::string &_filename,
      std::vector<gazebo::SystemPluginPtr> &_plugins);

  /// \internal
  /// \brief Setup the based gazebo system.
  /// \param[in] _prefix Prefix name, usually "client" or "server"
  /// \param[in] _argc Argument count
  /// \param[in] _argv Argument array
  /// \param[in] _plugins Vector of plugins to process
  /// \return True on success.
  bool setup(const std::string &_prefix, int _argc, char **_argv,
      std::vector<gazebo::SystemPluginPtr> &_plugins);
}
#endif
