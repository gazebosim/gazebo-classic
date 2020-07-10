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

#include "gazebo/common/Timer.hh"

#include "gazebo/sensors/sensors.hh"
#include "gazebo/sensors/LogicalCameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class LogicalCameraSensor : public ServerFixture
{
};

/////////////////////////////////////////////////
void OnNewUpdate(int* _msgCounter)
{
  *_msgCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, GroundPlane)
{
  Load("worlds/logical_camera.world");

  // Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  sensors::LogicalCameraSensorPtr cam = std::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  // Check that the camera parameters are correct
  EXPECT_NEAR(cam->Near(), 0.55, 1e-4);
  EXPECT_NEAR(cam->Far(), 5.0, 1e-4);
  EXPECT_NEAR(cam->HorizontalFOV().Radian(), 1.04719755, 1e-4);
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
TEST_F(LogicalCameraSensor, TopicNotSpecified)
{
  Load("worlds/logical_camera.world");

  // Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  sensors::LogicalCameraSensorPtr cam = std::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  EXPECT_EQ("~/box/link/logical_camera/models", cam->Topic());
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, TopicSpecified)
{
  Load("test/worlds/logical_camera_specify_topic.world");

  // Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  sensors::LogicalCameraSensorPtr cam = std::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  EXPECT_EQ("/publish/to/this/topic", cam->Topic());
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, Box)
{
  Load("worlds/logical_camera.world");

  // Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  // Get the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get the model that has the logical camera
  physics::ModelPtr cameraModel = world->ModelByName("box");
  ASSERT_TRUE(cameraModel != NULL);

  // Get the logical camera sensor
  sensors::LogicalCameraSensorPtr cam = std::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  // Force the update
  cam->Update(true);

  // We should now detect the ground plane
  ASSERT_EQ(cam->Image().model_size(), 1);
  EXPECT_EQ(cam->Image().model(0).name(), "ground_plane");

  // Insert box
  SpawnBox("spawn_box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(2, 0, 0.5),
      ignition::math::Vector3d::Zero);
  cam->Update(true);

  ASSERT_EQ(cam->Image().model_size(), 2);
  EXPECT_EQ(cam->Image().model(1).name(), "spawn_box");

  // Rotate the model, which should move "spawn_box" out of the frustum
  cameraModel->SetWorldPose(ignition::math::Pose3d(0, 0, 0, 0, 0, 1.5707));
  cam->Update(true);
  ASSERT_EQ(cam->Image().model_size(), 1);
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, NestedModels)
{
  Load("test/worlds/logical_camera_nested_models.world");

  // Wait until the sensor has been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  // Get the logical camera sensor
  sensors::LogicalCameraSensorPtr cam = std::dynamic_pointer_cast<
    sensors::LogicalCameraSensor>(sensors::get_sensor("logical_camera"));
  ASSERT_TRUE(cam != NULL);

  // Force the update
  cam->Update(true);

  // We should now detect the nested model
  ASSERT_EQ(1, cam->Image().model_size());
  EXPECT_EQ("base_target::nested_target", cam->Image().model(0).name());
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraSensor, StrictUpdateRate)
{
  LoadArgs(" --lockstep worlds/logical_camera_strict_rate.world");

  // Wait until the sensors have been initialized
  while (!sensors::SensorManager::Instance()->SensorsInitialized())
    common::Time::MSleep(1000);

  std::string cameraName = "logical_camera_sensor";
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::LogicalCameraSensorPtr logCamSensor =
    std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(sensor);
  ASSERT_TRUE(logCamSensor != NULL);

  int msgCount = 0;
  event::ConnectionPtr c = logCamSensor->ConnectUpdated(
        std::bind(&::OnNewUpdate, &msgCount));
  common::Timer timer;
  timer.Start();

  // how many msgs produced for 5 seconds (in simulated clock domain)
  double updateRate = logCamSensor->UpdateRate();
  int totalMsgs = 5 * updateRate;
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  double simT0 = 0.0;

  while (msgCount < totalMsgs)
  {
    // An approximation of when we receive the first image. In reality one
    // iteration before we receive the second image.
    if (msgCount == 0)
    {
      simT0 = world->SimTime().Double();
    }
    common::Time::MSleep(1);
  }

  // check that the obtained rate is the one expected
  double dt = world->SimTime().Double() - simT0;
  double rate = static_cast<double>(totalMsgs) / dt;
  gzdbg << "timer [" << dt << "] seconds rate [" << rate << "] fps\n";
  const double tolerance = 0.02;
  EXPECT_GT(rate, updateRate * (1 - tolerance));
  EXPECT_LT(rate, updateRate * (1 + tolerance));
  c.reset();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
