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
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

// this is the test fixture
class TransporterTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(TransporterTest, Transport)
{
  this->Load("worlds/transporter_test.world");

  // Get the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get the box model
  physics::ModelPtr boxModel = world->GetModel("box");
  ASSERT_TRUE(boxModel != NULL);

  // Make sure the starting location is correct.
  EXPECT_NEAR(boxModel->GetWorldPose().pos.x, 10.0, 1e-3);
  EXPECT_NEAR(boxModel->GetWorldPose().pos.y, 10.0, 1e-3);

  // Move the box onto the auto transport pad
  boxModel->SetWorldPose(math::Pose(100.5, 0, 0.5, 0, 0, 0));
  common::Time::Sleep(common::Time(1, 0));

  // Check that the box transported to the correct location
  EXPECT_NEAR(boxModel->GetWorldPose().pos.x, 0.0, 1e-3);
  EXPECT_NEAR(boxModel->GetWorldPose().pos.y, 0.0, 1e-3);

  // Move the box away
  boxModel->SetWorldPose(math::Pose(10, 10, 0.5, 0, 0, 0));
  common::Time::Sleep(common::Time(1, 0));

  // Check that the box is in the correct location
  EXPECT_NEAR(boxModel->GetWorldPose().pos.x, 10.0, 1e-3);
  EXPECT_NEAR(boxModel->GetWorldPose().pos.y, 10.0, 1e-3);

  // Move the box to the manual transporter pad
  boxModel->SetWorldPose(math::Pose(-100.5, 0, 0.5, 0, 0, 0));
  common::Time::Sleep(common::Time(1, 0));

  // Check that the box is in the correct location
  EXPECT_NEAR(boxModel->GetWorldPose().pos.x, -100.5, 1e-3);
  EXPECT_NEAR(boxModel->GetWorldPose().pos.y, 0, 1e-3);

  // Trigger the transporter
  {
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to a Gazebo topic
    gazebo::transport::PublisherPtr pub =
      node->Advertise<gazebo::msgs::GzString>("~/transporter");

    // Wait for a subscriber to connect
    pub->WaitForConnection();

    // Convert to a pose message
    gazebo::msgs::GzString msg;
    msg.set_data("pad2");
    pub->Publish(msg);
  }

  common::Time::Sleep(common::Time(1, 0));

  // Check that the box transported to the correct location
  EXPECT_NEAR(boxModel->GetWorldPose().pos.x, 0, 1e-3);
  EXPECT_NEAR(boxModel->GetWorldPose().pos.y, 0, 1e-3);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
