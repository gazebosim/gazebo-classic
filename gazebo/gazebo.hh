/*
 * Copyright (C) 2012-1013 Open Source Robotics Foundation
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
  void print_version();
  void add_plugin(const std::string &_filename);

  bool load(int _argc = 0, char **_argv = 0);
  bool init();
  void run();
  void stop();
  void fini();

  /// \brief Find a file in the gazebo search paths
  std::string find_file(const std::string &_file);
}

#endif
