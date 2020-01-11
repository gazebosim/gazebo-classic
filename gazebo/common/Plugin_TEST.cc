/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "gazebo/common/Plugin.hh"
#include "test/util.hh"
#include "test_config.h"

using namespace gazebo;

class PluginTest : public gazebo::testing::AutoLogFixture
{
  protected: virtual void SetUp()
  {
    gazebo::testing::AutoLogFixture::SetUp();

    // Register plugin paths in build directory

    boost::filesystem::path path;
    path = PROJECT_BINARY_PATH;
    path /= "plugins";
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(path.string());
  }
};

// helper function that gives correct prefix and extension for the OS
std::string expectedFilename(const std::string &_pluginName)
{
#ifdef _WIN32
  return _pluginName + ".dll";
#elif __APPLE__
  return "lib" + _pluginName + ".dylib";
#endif
  return "lib" + _pluginName + ".so";
}

TEST_F(PluginTest, LoadModelPlugin)
{
  ModelPluginPtr plugin = ModelPlugin::Create("libBuoyancyPlugin.so",
                                              "pluginInterfaceTest");
  ASSERT_TRUE(plugin != nullptr);

  EXPECT_EQ(plugin->GetType(), PluginType::MODEL_PLUGIN);
  EXPECT_EQ(plugin->GetFilename(), expectedFilename("BuoyancyPlugin"));
  EXPECT_EQ(plugin->GetHandle(), "pluginInterfaceTest");
}

TEST_F(PluginTest, LoadWorldPlugin)
{
  WorldPluginPtr plugin = WorldPlugin::Create("libArrangePlugin.so",
                                              "pluginInterfaceTest");
  ASSERT_TRUE(plugin != nullptr);

  EXPECT_EQ(plugin->GetType(), PluginType::WORLD_PLUGIN);
  EXPECT_EQ(plugin->GetFilename(), expectedFilename("ArrangePlugin"));
  EXPECT_EQ(plugin->GetHandle(), "pluginInterfaceTest");
}

TEST_F(PluginTest, LoadSensorPlugin)
{
  SensorPluginPtr plugin = SensorPlugin::Create("libContactPlugin.so",
                                              "pluginInterfaceTest");
  ASSERT_TRUE(plugin != nullptr);

  EXPECT_EQ(plugin->GetType(), PluginType::SENSOR_PLUGIN);
  EXPECT_EQ(plugin->GetFilename(), expectedFilename("ContactPlugin"));
  EXPECT_EQ(plugin->GetHandle(), "pluginInterfaceTest");
}

TEST_F(PluginTest, LoadSystemPlugin)
{
  SystemPluginPtr plugin = SystemPlugin::Create("libModelPropShop.so",
                                                "pluginInterfaceTest");
  ASSERT_TRUE(plugin != nullptr);

  EXPECT_EQ(plugin->GetType(), PluginType::SYSTEM_PLUGIN);
  EXPECT_EQ(plugin->GetFilename(), expectedFilename("ModelPropShop"));
  EXPECT_EQ(plugin->GetHandle(), "pluginInterfaceTest");
}

TEST_F(PluginTest, LoadVisualPlugin)
{
  VisualPluginPtr plugin = VisualPlugin::Create("libBlinkVisualPlugin.so",
                                                "pluginInterfaceTest");
  ASSERT_TRUE(plugin != nullptr);

  EXPECT_EQ(plugin->GetType(), PluginType::VISUAL_PLUGIN);
  EXPECT_EQ(plugin->GetFilename(), expectedFilename("BlinkVisualPlugin"));
  EXPECT_EQ(plugin->GetHandle(), "pluginInterfaceTest");
}


// TODO: The following test actually fails due to current unsafe implementation
// of plugin loading.
// See https://bitbucket.org/osrf/gazebo/issues/2267 for details.

/*
TEST_F(PluginTest, LoadModelPluginWrong)
{
  WorldPluginPtr plugin = WorldPlugin::Create("libPluginInterfaceTest.so",
                                              "pluginInterfaceTest");
  ASSERT_EQ(plugin, nullptr);
}
// */

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
