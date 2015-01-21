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
#include <gtest/gtest.h>
#include <stdio.h>
#include <string>
#include <boost/filesystem.hpp>

#include "gazebo/common/Console.hh"

///////////////////////////////////////////////////////////////////
// Create a temporary build folder
boost::filesystem::path createTempBuildFolder(const std::string &_prefix)
{
  boost::filesystem::path path = boost::filesystem::temp_directory_path();
  path /= boost::filesystem::unique_path(_prefix + "-%%%%-%%%%-%%%%-%%%%");
  boost::filesystem::create_directories(path);
  gzdbg << "mkdir " << path.string() << std::endl;
  return path;
}

///////////////////////////////////////////////////////////////////
// Get path to source folder with specified suffix
boost::filesystem::path getSourcePath(const std::string &_suffix)
{
  boost::filesystem::path path = CMAKE_SOURCE_DIR;
  path /= std::string("examples");
  path /= std::string("plugins");
  path /= _suffix;
  gzdbg << "source " << path.string() << std::endl;
  return path;
}

class ExamplePlugins : public ::testing::TestWithParam<const char*>
{
  /// \brief Build plugin in subfolder _name in a temporary build folder
  /// \param[in] _name Subfolder to build.
  public: void Build(const std::string &_name);
};

///////////////////////////////////////////////////////////////////
// Build plugin in subfolder _name in a temporary build folder
void ExamplePlugins::Build(const std::string &_name)
{
  // get a unique temporary build folder name
  boost::filesystem::path build = createTempBuildFolder(_name);

  // construct path of source folder
  boost::filesystem::path source = getSourcePath(_name);

  char cmd[1024];

  // cd build && cmake source
  snprintf(cmd, sizeof(cmd), "cd %s && cmake %s && make",
    build.c_str(), source.c_str());
  ASSERT_EQ(system(cmd), 0);

  // remove temporary folder
  gzdbg << "removing " << build.string() << std::endl;
  boost::filesystem::remove_all(build);
}

TEST_P(ExamplePlugins, Build)
{
  Build(GetParam());
}

INSTANTIATE_TEST_CASE_P(ExamplePlugins, ExamplePlugins, ::testing::Values(
  "animate_joints"
  , "animate_pose"
  , "factory"
  , "hello_world"
  , "model_push"
  , "parameters"
  , "projector"
  , "system_gui_plugin"
  , "world_edit"
  , "gui_overlay_plugin_spawn"
  , "gui_overlay_plugin_time"
));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
