/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
boost::filesystem::path getSourcePath(const std::string &_folder,
                                      const std::string &_suffix)
{
  boost::filesystem::path path = CMAKE_SOURCE_DIR;
  path /= std::string("examples");
  path /= _folder;
  path /= _suffix;
  gzdbg << "source " << path.string() << std::endl;
  return path;
}

///////////////////////////////////////////////////////////////////
class ExamplesBuild : public ::testing::TestWithParam<const char*>
{
  /// \brief Build code in subfolder _type/_name in a temporary build folder
  /// \param[in] _type Type of example to build (plugins, stand_alone).
  /// \param[in] _name Subfolder to build.
  public: void Build(const std::string &_type, const std::string &_name);
};

// Fixture for building example plugins
class ExamplesBuild_Plugins: public ExamplesBuild {};

// Fixture for building example stand_alone applications
class ExamplesBuild_Standalone: public ExamplesBuild {};

///////////////////////////////////////////////////////////////////
// Build code in subfolder _type/_name in a temporary build folder
void ExamplesBuild::Build(const std::string &_type, const std::string &_name)
{
  // get a unique temporary build folder name
  boost::filesystem::path build = createTempBuildFolder(_name);

  // construct path of source folder
  boost::filesystem::path source = getSourcePath(_type, _name);

  char cmd[1024];

  // cd build && cmake source
  snprintf(cmd, sizeof(cmd), "cd %s && cmake %s && make",
    build.c_str(), source.c_str());
  ASSERT_EQ(system(cmd), 0);

  // remove temporary folder
  gzdbg << "removing " << build.string() << std::endl;
  boost::filesystem::remove_all(build);
}

///////////////////////////////////////////////////////////////////
TEST_P(ExamplesBuild_Plugins, Plugins)
{
  Build("plugins", GetParam());
}

///////////////////////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(Plugins, ExamplesBuild_Plugins, ::testing::Values(
  "animate_joints"
  , "animate_pose"
  , "factory"
  , "gui_overlay_plugin_spawn"
  , "gui_overlay_plugin_time"
  , "hello_world"
  , "model_push"
  , "model_move"
  , "parameters"
  , "projector"
  , "system_gui_plugin"
  , "world_edit"
));

///////////////////////////////////////////////////////////////////
TEST_P(ExamplesBuild_Standalone, Standalone)
{
  Build("stand_alone", GetParam());
}

///////////////////////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(Standalone, ExamplesBuild_Standalone, ::testing::Values(
  "actuator"
  // , "animated_box"
  // , "arrange"
  , "clone_simulation"
  , "custom_main"
  , "custom_main_pkgconfig"
  // , "listener"
  // , "publisher"
  , "test_fixture"
));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
