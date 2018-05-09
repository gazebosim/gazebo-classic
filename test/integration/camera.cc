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

#include "gazebo/test/ServerFixture.hh"

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
  SpawnBox("box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(10, 10, 1),
      ignition::math::Vector3d::Zero);

  ignition::math::Pose3d cameraStartPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraStartPose.Pos(), cameraStartPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor("test_camera");
  EXPECT_TRUE(sensor != NULL);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor != NULL);
  rendering::CameraPtr camera = camSensor->Camera();
  ASSERT_TRUE(camera != NULL);

  // Make sure the sensor is at the correct initial pose
  EXPECT_EQ(camera->WorldPose(), cameraStartPose);

  SetPause(true);

  // Tell the camera to follow the box. The camera should move toward the
  // box.
  msgs::CameraCmd msg;
  msg.set_follow_model("box");

  transport::NodePtr node(new transport::Node());
  node->Init("default");

  transport::PublisherPtr pub = node->Advertise<msgs::CameraCmd>(
      "~/test_camera_model/body/test_camera/cmd");

  pub->WaitForConnection();
  pub->Publish(msg, true);

  world->Step(1000);

  // Make sure the sensor is at the correct initial pose
  EXPECT_TRUE(camera->WorldPose() != cameraStartPose);

  EXPECT_NEAR(camera->WorldPose().Pos().X(), 4.3, 0.1);
  EXPECT_NEAR(camera->WorldPose().Pos().Y(), 4.3, 0.1);
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
  SpawnBox("box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Vector3d::Zero);

  ignition::math::Pose3d cameraStartPose(0, 0, 0, 0, 0, 0);

  std::string cameraName = "test_camera";
  // Spawn a camera facing the box
  SpawnCamera("test_camera_model", cameraName,
      cameraStartPose.Pos(), cameraStartPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  ASSERT_TRUE(sensor != NULL);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor != NULL);
  rendering::CameraPtr camera = camSensor->Camera();
  ASSERT_TRUE(camera != NULL);

  // this makes sure a world step will trigger the camera render update
  sensor->SetUpdateRate(1000);

  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene != NULL);
  // Make sure the camera is at the correct initial pose
  EXPECT_EQ(camera->WorldPose(), cameraStartPose);

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

  physics::ModelPtr box = world->ModelByName("box");
  ASSERT_TRUE(box != NULL);

  // move the box behind the camera and it should not be visible to the camera
  ignition::math::Pose3d pose(-1, 0, 0.5, 0, 0, 0);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->WorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->WorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->Name()));

  // move the box to the left of the camera and it should not be visible
  pose = ignition::math::Pose3d(0, -1, 0.5, 0, 0, 0);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->WorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->WorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->Name()));

  // move the box to the right of the camera with some rotations,
  // it should still not be visible.
  pose = ignition::math::Pose3d(0, 1, 0.5, 0, 0, 1.57);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->WorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->WorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->Name()));

  // rotate the camera counter-clockwise to see the box
  camera->Yaw(ignition::math::Angle(1.57));
  EXPECT_TRUE(camera->IsVisible(visual));
  EXPECT_TRUE(camera->IsVisible(visual->Name()));

  // move the box up and let it drop. The camera should not see the box
  // initially but the box should eventually move into the camera view
  // as it falls
  pose = ignition::math::Pose3d(0, 1, 5.5, 0, 0, 1.57);
  box->SetWorldPose(pose);
  world->Step(1);
  sleep = 0;
  maxSleep = 10;
  while (visual->WorldPose() != pose && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_TRUE(visual->WorldPose() == pose);
  world->Step(1);
  EXPECT_TRUE(!camera->IsVisible(visual));
  EXPECT_TRUE(!camera->IsVisible(visual->Name()));

  sleep = 0;
  maxSleep = 100;
  while (!camera->IsVisible(visual) && sleep < maxSleep)
  {
    world->Step(10);
    sleep++;
  }
  EXPECT_TRUE(camera->IsVisible(visual));
  EXPECT_TRUE(camera->IsVisible(visual->Name()));
}

/////////////////////////////////////////////////
// Spawn a mesh in front of camera and test ray triangle intersection
TEST_F(CameraTest, RayTriangleIntersection)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  ignition::math::Pose3d cameraStartPose(-1, 0, 0.02, 0, 0, 0);

  // Spawn a camera for ray intersection test
  SpawnCamera("test_camera_model", "test_camera",
      cameraStartPose.Pos(), cameraStartPose.Rot().Euler());

  // Spawn a cordless drill
  SpawnModel("model://cordless_drill");
  WaitUntilEntitySpawn("drill", 100, 50);

  // verify ray triangle intersection in rendering thread
  bool verified = false;
  auto testRayIntersection = [&]()
  {
    // get scene
    rendering::ScenePtr scene = rendering::get_scene();
    ASSERT_TRUE(scene != nullptr);

    // wait until the camera is created in scene
    rendering::CameraPtr camera = scene->GetCamera(
        "test_camera_model::body::test_camera");
    if (!camera)
      return;

    // wait until the drill is created in scene
    rendering::VisualPtr drillVis = scene->GetVisual("drill");
    if (!drillVis)
      return;

    // wait until the camera is in place
    if (camera->WorldPose() != cameraStartPose)
      return;

    // sanity check
    if (!camera->IsVisible(drillVis))
      return;

    // test ray intersection with a mesh
    ignition::math::Vector2i drillImgPos =
        camera->Project(drillVis->WorldPose().Pos() +
                        ignition::math::Vector3d(0, 0, 0.05));

    ignition::math::Vector3d intersectPos;
    bool intersect = scene->FirstContact(camera, drillImgPos, intersectPos);

    // camera ray should intersect with a triangle on mesh
    EXPECT_TRUE(intersect);
    EXPECT_GT(intersectPos.X(), -1);
    EXPECT_LT(intersectPos.X(), 0);
    EXPECT_TRUE(ignition::math::equal(intersectPos.Y(), 0.0, 1e-4));
    EXPECT_GT(intersectPos.Z(), 0);
    EXPECT_LT(intersectPos.Z(), 0.05);

    // cast a ray so that it's within the bounding box of the mesh but doesn't
    // intersect with any triangles
    ignition::math::Vector2i emptyImgPos =
        camera->Project(drillVis->WorldPose().Pos() +
                        ignition::math::Vector3d(0, 0, 0.1));

    // ray should not intersect with any triangles on mesh
    intersect = scene->FirstContact(camera, emptyImgPos, intersectPos);
    EXPECT_FALSE(intersect);
    EXPECT_EQ(ignition::math::Vector3d::Zero, intersectPos);

    verified = true;
  };
  auto connection =
      event::Events::ConnectPreRender(std::bind(testRayIntersection));

  // wait until the verification has been done in the rendering thread
  int sleep = 0;
  int maxSleep = 50;
  while (sleep++ < maxSleep)
  {
    common::Time::MSleep(100);
    if (verified)
      break;
  }
  EXPECT_TRUE(verified);

  connection.reset();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
