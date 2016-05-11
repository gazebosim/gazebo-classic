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

#include <gtest/gtest.h>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class CameraSensor_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(CameraSensor_TEST, CreateCamera)
{
  this->Load("worlds/empty.world");
  this->SpawnCamera("camera", "camera", ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);

  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the camera sensor
  std::string sensorName = "default::camera::body::camera";

  // Get a pointer to the camera sensor
  sensors::CameraSensorPtr sensor =
     std::dynamic_pointer_cast<sensors::CameraSensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_TRUE(sensor->IsActive());

  EXPECT_EQ(sensor->ImageWidth(), 320u);
  EXPECT_EQ(sensor->ImageHeight(), 240u);
  EXPECT_FALSE(sensor->Topic().empty());

  // wait for camera images
  int sleep = 0;
  int maxSleep = 20;
  while (sleep < maxSleep && !sensor->ImageData())
  {
    sleep++;
    common::Time::MSleep(100);
  }
  EXPECT_TRUE(sensor->ImageData() != nullptr);

  // Remove the sensor
  std::string sensorScopedName = sensor->ScopedName();
  sensors::SensorManager::Instance()->RemoveSensor(sensorScopedName);

  // wait for the sensor to be removed
  sleep = 0;
  while (sensors::SensorManager::Instance()->GetSensor(sensorScopedName)
      && sleep < maxSleep)
  {
    sleep++;
    common::Time::MSleep(100);
  }
  EXPECT_TRUE(sensors::SensorManager::Instance()->GetSensor(sensorScopedName)
      == nullptr);

  // Check that sensor is invalid after being removed.
  EXPECT_EQ(sensor->ImageWidth(), 0u);
  EXPECT_EQ(sensor->ImageHeight(), 0u);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
