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

#include <ignition/msgs/plugin_v.pb.h>

#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"
#include "gazebo/common/URI.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Model.hh"

using namespace gazebo;

class ModelTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(ModelTest, Scale)
{
  // Load a world
  this->Load("worlds/empty.world", true);
  auto world = physics::get_world("default");

  // Create a base to be the model's parent
  physics::BasePtr basePtr;
  basePtr.reset(new physics::Base(physics::BasePtr()));
  basePtr->SetName("world_root_element");
  basePtr->SetWorld(world);

  // Create the model
  physics::ModelPtr modelPtr(new physics::Model(basePtr));
  EXPECT_TRUE(modelPtr != NULL);

  // Check default scale
  EXPECT_EQ(modelPtr->Scale(), ignition::math::Vector3d(1, 1, 1));

  // Set scale
  ignition::math::Vector3d scale(0.5, 2, 3);
  modelPtr->SetScale(scale);

  // Check scale
  EXPECT_EQ(modelPtr->Scale(), scale);

  // Cleanup
  modelPtr.reset();
}

//////////////////////////////////////////////////
TEST_F(ModelTest, NestedModelSensorScopedName)
{
  this->Load("test/worlds/deeply_nested_models.world", true);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto model = world->ModelByName("model_03");
  ASSERT_TRUE(model != nullptr);

  gzmsg << "Get a scoped sensor name" << std::endl;

  const std::vector<std::string> imuScopedName =
      model->SensorScopedName("imu_sensor");
  ASSERT_EQ(imuScopedName.size(), static_cast<unsigned int>(1));
  EXPECT_EQ(imuScopedName[0],
    "default::model_00::model_01::model_02::model_03::imu_link::imu_sensor");

  gzmsg << "Get a nonexistent scoped sensor name" << std::endl;

  const std::vector<std::string> noScopedName =
      model->SensorScopedName("no_sensor");
  ASSERT_EQ(noScopedName.size(), static_cast<unsigned int>(0));
}

//////////////////////////////////////////////////
TEST_F(ModelTest, PluginInfoFailures)
{
  this->Load("worlds/shapes.world", true);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto model = world->ModelByName("box");
  ASSERT_TRUE(model != nullptr);

  ignition::msgs::Plugin_V plugins;
  bool success;
  common::URI pluginUri;

  gzmsg << "Model has no plugins" << std::endl;
  {
    pluginUri.Parse("data://world/default/model/box/plugin/");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_TRUE(success);
    EXPECT_EQ(plugins.plugins_size(), 0);
  }

  gzmsg << "Wrong world" << std::endl;
  {
    pluginUri.Parse("data://world/wrong/model/box/plugin/");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Wrong model" << std::endl;
  {
    pluginUri.Parse("data://world/default/model/cone/plugin/");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Invalid URI" << std::endl;
  {
    pluginUri = common::URI("tell me about your plugins");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Unhandled URI" << std::endl;
  {
    pluginUri.Parse("data://world/default/plugin/");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Inexistent nested model" << std::endl;
  {
    pluginUri.Parse(
        "data://world/default/model/box/model/box_in_a_box/plugin");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Incomplete URI" << std::endl;
  {
    pluginUri.Parse("data://world/default/model/box");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }
}

//////////////////////////////////////////////////
TEST_F(ModelTest, ModelPluginInfo)
{
  this->Load("worlds/underwater.world", true);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto model = world->ModelByName("submarine");
  ASSERT_TRUE(model != nullptr);

  ignition::msgs::Plugin_V plugins;
  bool success;
  common::URI pluginUri;

  gzmsg << "Get an existing plugin" << std::endl;
  {
    pluginUri.Parse(
        "data://world/default/model/submarine/plugin/submarine_propeller_3");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_TRUE(success);
    ASSERT_EQ(plugins.plugins_size(), 1);
    EXPECT_EQ(plugins.plugins(0).name(), "submarine_propeller_3");
  }

  gzmsg << "Get all plugins" << std::endl;
  {
    pluginUri.Parse("data://world/default/model/submarine/plugin/");
    model->PluginInfo(pluginUri, plugins, success);

    EXPECT_TRUE(success);
    ASSERT_EQ(plugins.plugins_size(), 5);
    EXPECT_EQ(plugins.plugins(0).name(), "submarine_propeller_1");
    EXPECT_EQ(plugins.plugins(1).name(), "submarine_propeller_2");
    EXPECT_EQ(plugins.plugins(2).name(), "submarine_propeller_3");
    EXPECT_EQ(plugins.plugins(3).name(), "submarine_propeller_4");
    EXPECT_EQ(plugins.plugins(4).name(), "buoyancy");
  }
}

//////////////////////////////////////////////////
TEST_F(ModelTest, NestedModelPluginInfo)
{
  this->Load("test/worlds/deeply_nested_models.world", true);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto model = world->ModelByName("model_00");
  ASSERT_TRUE(model != nullptr);

  ignition::msgs::Plugin_V plugins;
  bool success;
  common::URI pluginUri;

  gzmsg << "Get an existing plugin" << std::endl;

  pluginUri.Parse(
      "data://world/default/model/model_00/model/model_01/model/model_02/"
      "model/model_03/plugin/region_event_box");
  model->PluginInfo(pluginUri, plugins, success);

  EXPECT_TRUE(success);
  ASSERT_EQ(plugins.plugins_size(), 1);
  EXPECT_EQ(plugins.plugins(0).name(), "region_event_box");
}

//////////////////////////////////////////////////
TEST_F(ModelTest, BoundingBox)
{
  this->Load("worlds/another_box.world", true);

  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto model = world->ModelByName("box");
  ASSERT_TRUE(model != nullptr);

  EXPECT_EQ(ignition::math::Box(-10.5, -20.5, -30.5, -9.5, -19.5, -29.5),
            model->BoundingBox());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
