/*
 * Copyright 2013 Open Source Robotics Foundation
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
class CameraFollow : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test the at camera follow (gz camera -f <model> -c <camera>) moves
// the camera
TEST_F(CameraFollow, Follow)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  // Spawn a box to follow.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene);

  rendering::CameraPtr camera = scene->GetCamera("test_camera");
  ASSERT_TRUE(camera);

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
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
