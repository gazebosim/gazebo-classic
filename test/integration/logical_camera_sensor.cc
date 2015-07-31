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

#include "gazebo/sensors/sensors.hh"
#include "gazebo/sensors/LogicalCameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class LogicalCameraSensor : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, GroundPlane)
{
  Load("worlds/logical_camera.world");

  /// \brief Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  sensors::LogicalCameraSensorPtr cam = boost::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  // Check that the camera parameters are correct
  EXPECT_NEAR(cam->Near(), 0.55, 1e-4);
  EXPECT_NEAR(cam->Far(), 5.0, 1e-4);
  EXPECT_NEAR(cam->FOV().Radian(), 1.04719755, 1e-4);
  EXPECT_NEAR(cam->AspectRatio(), 1.778, 1e-4);
  EXPECT_TRUE(cam->IsActive());

  // Update, but do not force the update
  cam->Update(false);
  EXPECT_EQ(cam->Image().model_size(), 0);

  // Force the update
  cam->Update(true);

  // We should now detect the ground plane
  ASSERT_EQ(cam->Image().model_size(), 1);
  EXPECT_EQ(cam->Image().model(0).name(), "ground_plane");
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, Box)
{
  Load("worlds/logical_camera.world");

  /// \brief Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  // Get the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get the model that has the logical camera
  physics::ModelPtr cameraModel = world->GetModel("box");
  ASSERT_TRUE(cameraModel != NULL);

  // Get the logical camera sensor
  sensors::LogicalCameraSensorPtr cam = boost::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  // Force the update
  cam->Update(true);

  // We should now detect the ground plane
  ASSERT_EQ(cam->Image().model_size(), 1);
  EXPECT_EQ(cam->Image().model(0).name(), "ground_plane");

  // Insert box
  SpawnBox("spawn_box", math::Vector3(1, 1, 1), math::Vector3(2, 0, 0.5),
      math::Vector3::Zero);
  cam->Update(true);

  ASSERT_EQ(cam->Image().model_size(), 2);
  EXPECT_EQ(cam->Image().model(1).name(), "spawn_box");

  // Rotate the model, which should move "spawn_box" out of the frustum
  cameraModel->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 1.5707));
  cam->Update(true);
  ASSERT_EQ(cam->Image().model_size(), 1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
