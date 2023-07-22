/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/common/CommonIface.hh"

using namespace gazebo;
class PluginTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(PluginTest, ModelExceptionConstructor)
{
  Load("worlds/exception_model_plugin_constructor_test.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  world->Step(100);

  char *home = getenv(HOMEDIR);
  ASSERT_TRUE(home);

  boost::filesystem::path path(home);
  path /= "/.gazebo/server-11345/default.log";

  // Open the log file, and read back the string
  std::ifstream ifs(path.string().c_str(), std::ios::in);
  std::string loggedString;

  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  EXPECT_TRUE(loggedString.find(
        "Exception occured in the constructor of plugin with name") !=
      std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(PluginTest, ModelExceptionInit)
{
  Load("worlds/exception_model_plugin_init_test.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  world->Step(100);

  char *home = getenv(HOMEDIR);
  ASSERT_TRUE(home);

  boost::filesystem::path path(home);
  path /= "/.gazebo/server-11345/default.log";

  // Open the log file, and read back the string
  std::ifstream ifs(path.string().c_str(), std::ios::in);
  std::string loggedString;

  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  EXPECT_TRUE(loggedString.find(
        "Exception occured in the Init function of plugin with name") !=
      std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(PluginTest, ModelExceptionLoad)
{
  Load("worlds/exception_model_plugin_load_test.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  world->Step(100);

  char *home = getenv(HOMEDIR);
  ASSERT_TRUE(home);

  boost::filesystem::path path(home);
  path /= "/.gazebo/server-11345/default.log";

  // Open the log file, and read back the string
  std::ifstream ifs(path.string().c_str(), std::ios::in);
  std::string loggedString;

  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  EXPECT_TRUE(loggedString.find(
        "Exception occured in the Load function of plugin with name") !=
      std::string::npos);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
