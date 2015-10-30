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
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class CameraSensor_TEST : public ServerFixture
{
};


TEST_F(CameraSensor_TEST, CreateCamera)
{
  Load("worlds/camera.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the Ray sensor
  std::string sensorName = "default::camera::link::camera";

  // Get a pointer to the Ray sensor
  sensors::CameraSensorPtr sensor =
     boost::dynamic_pointer_cast<sensors::CameraSensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);


/*  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(cameraSensorString, sdf);

  physics::WorldPtr world = physics::get_world("default");
  physics::ModelPtr model = world->GetModel("ground_plane");
  physics::LinkPtr link = model->GetLink("link");

  // Create the camera sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", link->GetId());

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::camera"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the camera sensor
  sensors::CameraSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_TRUE(sensor != NULL);*/

  // Update the sensor
  // sensor->Update(true);

  EXPECT_TRUE(sensor->IsActive());

  EXPECT_EQ(sensor->GetImageWidth(), 320);
  EXPECT_EQ(sensor->GetImageHeight(), 240);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
