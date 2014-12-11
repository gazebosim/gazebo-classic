/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include "ServerFixture.hh"

using namespace gazebo;
class CameraTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test that camera follow (gz camera -f <model> -c <camera>) moves
// the camera
TEST_F(CameraTest, Follow)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Spawn a box to follow.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  rendering::CameraPtr camera = scene->GetCamera("test_camera");
  ASSERT_TRUE(camera != NULL);

  // Make sure the sensor is at the correct initial pose
  EXPECT_EQ(camera->GetWorldPose(), cameraStartPose);

  SetPause(true);

  // Tell the camera to follow the box. The camera should move toward the
  // box.
  msgs::CameraCmd msg;
  msg.set_follow_model("box");

  transport::NodePtr node(new transport::Node());
  node->Init("default");

  transport::PublisherPtr pub =
    node->Advertise<msgs::CameraCmd>("~/test_camera/cmd");

  pub->WaitForConnection();
  pub->Publish(msg, true);

  world->Step(1000);

  // Make sure the sensor is at the correct initial pose
  EXPECT_TRUE(camera->GetWorldPose() != cameraStartPose);

  EXPECT_NEAR(camera->GetWorldPose().pos.x, 4.3, 0.1);
  EXPECT_NEAR(camera->GetWorldPose().pos.y, 4.3, 0.1);
}

/////////////////////////////////////////////////
// \brief Test the camera IsVisible function
TEST_F(CameraTest, Visible)
{
  Load("worlds/empty.world", true);

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Spawn a box.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(1, 0, 0.5),
      math::Vector3(0, 0, 0));

  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);

  std::string cameraName = "test_camera";
  // Spawn a camera facing the box
  SpawnCamera("test_camera_model", cameraName,
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  ASSERT_TRUE(sensor != NULL);
  // this makes sure a world step will trigger the camera render update
  sensor->SetUpdateRate(1000);

  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene != NULL);
  rendering::CameraPtr camera = scene->GetCamera(cameraName);
  ASSERT_TRUE(camera != NULL);

  // Make sure the camera is at the correct initial pose
  EXPECT_EQ(camera->GetWorldPose(), cameraStartPose);

  int sleep = 0;
  int maxSleep = 5;
  rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    visual = scene->GetVisual("box::body");
    common::Time::MSleep(100);
    sleep++;
  }
  ASSERT_TRUE(visual != NULL);

  // box should be visible to the camera.
  EXPECT_TRUE(camera->IsVisible(visual));

  physics::ModelPtr box = world->GetModel("box");
  ASSERT_TRUE(box != NULL);

  // move the box behind the camera and it should not be visible to the camera
  math::Pose pose = math::Pose(-1, 0, 0.5, 0, 0, 0);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->GetWorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->GetWorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->GetName()));

  // move the box to the left of the camera and it should not be visible
  pose = math::Pose(0, -1, 0.5, 0, 0, 0);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->GetWorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->GetWorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->GetName()));

  // move the box to the right of the camera with some rotations,
  // it should still not be visible.
  pose = math::Pose(0, 1, 0.5, 0, 0, 1.57);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->GetWorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->GetWorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->GetName()));

  // rotate the camera counter-clockwise to see the box
  camera->Yaw(math::Angle(1.57));
  EXPECT_TRUE(camera->IsVisible(visual));
  EXPECT_TRUE(camera->IsVisible(visual->GetName()));

  // move the box up and let it drop. The camera should not see the box
  // initially but the box should eventually move into the camera view
  // as it falls
  pose = math::Pose(0, 1, 5.5, 0, 0, 1.57);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->GetWorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->GetWorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->GetName()));

  sleep = 0;
  maxSleep = 100;
  while (!camera->IsVisible(visual) && sleep < maxSleep)
  {
    world->Step(10);
    sleep++;
  }
  EXPECT_TRUE(camera->IsVisible(visual));
  EXPECT_TRUE(camera->IsVisible(visual->GetName()));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
