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

#include <gtest/gtest.h>
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class SensorManager_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test SensorManager initialization
TEST_F(SensorManager_TEST, InitEmpty)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  EXPECT_TRUE(mgr->SensorsInitialized());
  sensors::Sensor_V sensors = mgr->GetSensors();
  size_t size = 0;
  EXPECT_EQ(sensors.size(), size);
}

/////////////////////////////////////////////////
/// \brief Test that sensors are actually created and updated.
TEST_F(SensorManager_TEST, Data)
{
  sensors::SensorPtr sensor;

  // Load in a world with cameras and lasers
  Load("worlds/test_camera_laser.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  EXPECT_TRUE(mgr->SensorsInitialized());

  // Make sure we have the right number of sensors
  size_t sensorCount = 4;
  sensors::Sensor_V sensors = mgr->GetSensors();

  int i = 0;
  while (sensors.size() != sensorCount && i < 100)
  {
    gazebo::common::Time::MSleep(100);
    sensors.clear();
    sensors = mgr->GetSensors();
    gzdbg << "Sensor Count. Actual[" << sensors.size() << "] Expected["
          << sensorCount << "]\n";
    ++i;
  }

  EXPECT_LT(i, 100);
  EXPECT_EQ(sensors.size(), sensorCount);

  // Get the current simulation time
  common::Time time = physics::get_world()->GetSimTime();

  // Wait for 1 second
  for (unsigned int i = 0; i < 10; ++i)
  {
    common::Time::MSleep(100);
  }

  // Get each sensor, and make sure that it has been updated
  {
    sensor = mgr->GetSensor("default::camera_1::link::camera");
    ASSERT_TRUE(sensor != nullptr);
    EXPECT_TRUE(sensor->LastMeasurementTime() > time);

    sensor = mgr->GetSensor("default::camera_2::link::camera");
    ASSERT_TRUE(sensor != nullptr);
    EXPECT_TRUE(sensor->LastMeasurementTime() > time);

    sensor = mgr->GetSensor("default::laser_1::link::laser");
    ASSERT_TRUE(sensor != nullptr);
    EXPECT_TRUE(sensor->LastMeasurementTime() > time);

    sensor = mgr->GetSensor("default::laser_2::link::laser");
    ASSERT_TRUE(sensor != nullptr);
    EXPECT_TRUE(sensor->LastMeasurementTime() > time);
  }
}

/////////////////////////////////////////////////
/// \brief Test SensorManager init and removal of sensors
TEST_F(SensorManager_TEST, InitRemove)
{
  // Load in the pr2
  Load("worlds/pr2.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  size_t sensorCount = 18;

  // Make sure we have the correct number of sensors.
  EXPECT_TRUE(mgr->SensorsInitialized());
  sensors::Sensor_V sensors = mgr->GetSensors();
  EXPECT_EQ(sensors.size(), sensorCount);

  std::vector<std::string> sensorNames;
  for (sensors::Sensor_V::iterator iter = sensors.begin();
       iter != sensors.end(); ++iter)
  {
    sensorNames.push_back((*iter)->Name());
  }

  // Try removing a few senors.
  for (std::vector<std::string>::iterator iter = sensorNames.begin();
       iter != sensorNames.end() && sensorCount > 10; ++iter)
  {
    mgr->RemoveSensor(*iter);

    --sensorCount;

    int i = 0;

    // Wait for a sensor manager update.
    while (mgr->GetSensors().size() > sensorCount)
    {
      gazebo::common::Time::MSleep(100);
      ++i;
    }

    EXPECT_LT(i, 100);
  }

  // Make sure the proper number of sensors have been removed
  EXPECT_EQ(mgr->GetSensors().size(), sensorCount);

  // Remove the rest of the sensors
  mgr->RemoveSensors();

  int i = 0;
  // Wait for a sensor manager update.
  while (mgr->GetSensors().size() > 0 && i < 100)
  {
    gazebo::common::Time::MSleep(100);
    ++i;
  }
  EXPECT_LT(i, 100);

  // Make sure all the sensors have been removed
  EXPECT_EQ(mgr->GetSensors().size(), size_t(0));

  printf("Done done\n");
}

//////////////////////////////////////////////////
TEST_F(SensorManager_TEST, SensorInfo)
{
  this->Load("worlds/depth_camera.world", true);

  // Get the SensorManager instance
  auto mgr = sensors::SensorManager::Instance();
  ASSERT_TRUE(mgr != nullptr);

  ignition::msgs::Sensor_V sensors;
  bool success;
  common::URI sensorUri;

  gzmsg << "Get an existing sensor" << std::endl;
  {
    sensorUri.Parse(
        "data://world/default/model/camera_model/link/my_link/sensor/camera");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_TRUE(success);
    EXPECT_EQ(sensors.sensors_size(), 1);
    EXPECT_EQ(sensors.sensors(0).name(), "camera");
  }
}

//////////////////////////////////////////////////
TEST_F(SensorManager_TEST, PluginInfo)
{
  this->Load("worlds/depth_camera.world", true);

  // Get the SensorManager instance
  auto mgr = sensors::SensorManager::Instance();
  ASSERT_TRUE(mgr != nullptr);

  ignition::msgs::Plugin_V plugins;
  bool success;
  common::URI pluginUri;

  gzmsg << "Get an existing plugin" << std::endl;
  {
    pluginUri.Parse("data://world/default/model/camera_model/"
      "link/my_link/sensor/camera/plugin/depth_camera_plugin");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_TRUE(success);
    EXPECT_EQ(plugins.plugins_size(), 1);
    EXPECT_EQ(plugins.plugins(0).name(), "depth_camera_plugin");
  }
}

//////////////////////////////////////////////////
TEST_F(SensorManager_TEST, PluginInfoFailures)
{
  this->Load("worlds/magnetometer.world", true);

  // Get the SensorManager instance
  auto mgr = sensors::SensorManager::Instance();

  ignition::msgs::Plugin_V plugins;
  bool success;
  gazebo::common::URI pluginUri;

  gzmsg << "Sensor has no plugins" << std::endl;
  {
    pluginUri.Parse(
        "data://world/default/model/magnetometerModel/link/link/sensor/"
        "magnetometer/plugin");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_TRUE(success);
    EXPECT_EQ(plugins.plugins_size(), 0);
  }

  gzmsg << "Wrong world" << std::endl;
  {
    pluginUri.Parse(
        "data://world/wrong/model/magnetometerModel/link/link/sensor/"
        "magnetometer/plugin");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Invalid URI" << std::endl;
  {
    pluginUri = gazebo::common::URI("tell me about your plugins");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Unhandled URI" << std::endl;
  {
    pluginUri.Parse("data://world/default/plugin/");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Incomplete URI" << std::endl;
  {
    pluginUri.Parse(
        "data://world/wrong/model/magnetometerModel/link/link/sensor/"
        "magnetometer");
    mgr->PluginInfo(pluginUri, plugins, success);

    EXPECT_FALSE(success);
  }
}

//////////////////////////////////////////////////
TEST_F(SensorManager_TEST, SensorInfoFailures)
{
  this->Load("worlds/shapes.world", true);

  // Get the SensorManager instance
  auto mgr = sensors::SensorManager::Instance();

  ignition::msgs::Sensor_V sensors;
  bool success;
  gazebo::common::URI sensorUri;

  gzmsg << "Link has no sensors" << std::endl;
  {
    sensorUri.Parse(
        "data://world/default/model/box/link/link/sensor/");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_TRUE(success);
    EXPECT_EQ(sensors.sensors_size(), 0);
  }

  gzmsg << "Wrong world" << std::endl;
  {
    sensorUri.Parse(
        "data://world/wrong/model/box/link/link/sensor/");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Wrong model" << std::endl;
  {
    sensorUri.Parse(
        "data://world/default/model/wrong/link/link/sensor/");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Invalid URI" << std::endl;
  {
    sensorUri = gazebo::common::URI("tell me about your sensors");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Unhandled URI" << std::endl;
  {
    sensorUri.Parse("data://world/default/model/box/link/link/visual/visual/");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_FALSE(success);
  }

  gzmsg << "Incomplete URI" << std::endl;
  {
    sensorUri.Parse("data://world/default/model/box/link/link");
    mgr->SensorInfo(sensorUri, sensors, success);

    EXPECT_FALSE(success);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
