/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

class TrackedVehiclesTest : public ServerFixture
{
  public: explicit TrackedVehiclesTest(const std::string &_worldFilename)
  {
    this->Load(_worldFilename, true);
    this->world = physics::get_world();

    this->node.reset(new transport::Node());
    this->node->Init();
  }

  protected: physics::WorldPtr world;
  protected: transport::NodePtr node;
};

class SimpleTrackedVehiclesTest : public TrackedVehiclesTest
{
 public:
  SimpleTrackedVehiclesTest()
    : TrackedVehiclesTest("worlds/tracked_vehicle_simple.world")
  {
    auto pub = this->node->Advertise<msgs::Factory>("~/factory");

    msgs::Factory msg;
    msg.set_clone_model_name("simple_tracked");
    msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(
        ignition::math::Vector3d(2, 3, 0.1),
        ignition::math::Quaterniond::Identity));

    pub->Publish(msg);

    this->WaitUntilEntitySpawn("simple_tracked_clone", 100, 100);
  }
};

class WheeledTrackedVehiclesTest : public TrackedVehiclesTest
{
 public:
  WheeledTrackedVehiclesTest()
    : TrackedVehiclesTest("worlds/tracked_vehicle_wheeled.world")
  {
    auto pub = this->node->Advertise<msgs::Factory>("~/factory");

    msgs::Factory msg;
    msg.set_clone_model_name("wheel_tracked");
    msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(
        ignition::math::Vector3d(2, 3, 0.1),
        ignition::math::Quaterniond::Identity));

    pub->Publish(msg);

    this->WaitUntilEntitySpawn("wheel_tracked_clone", 100, 100);
  }
};

// Verify that a model's world pose is near a specified pose.
void verifyPose(
    const physics::ModelPtr model,
    const ignition::math::Pose3d& pose)
{
  EXPECT_NEAR(model->WorldPose().Pos().X(), pose.Pos().X(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), pose.Pos().Y(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), pose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), pose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), pose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(), pose.Rot().Yaw(), 1e-1);
}

// Test that the SimpleTracked vehicle is moving as expected.
TEST_F(SimpleTrackedVehiclesTest, SimpleTracked)
{
  ASSERT_NE(this->world->ModelCount(), 0u);
  physics::ModelPtr model = this->world->ModelByName("simple_tracked");
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->GetName(), "simple_tracked");

  auto modelClone = this->world->ModelByName("simple_tracked_clone");
  ASSERT_NE(modelClone, nullptr);

  transport::PublisherPtr pub = this->node->Advertise<msgs::Twist>(
    "~/simple_tracked/cmd_vel_twist");
  pub->WaitForConnection();

  // Move the robot somewhere to free space without obstacles.
  model->SetWorldPose(ignition::math::Pose3d(10, 10, 0.1, 0, 0, 0));

  // Let the models settle down.
  this->world->Step(300);

  // Test straight driving - 1 sec driving, should move 1 meter forward.

  const auto startPose = model->WorldPose();
  const auto startPoseClone = modelClone->WorldPose();

  const double forwardSpeed = 1.0;
  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(),
              startPose.Pos().X() + forwardSpeed,
              1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), startPose.Pos().Y(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), startPose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), startPose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), startPose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(), startPose.Rot().Yaw(), 1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test rotation in place - 1 sec rotation, should turn 0.25 rad.

  const auto middlePose = model->WorldPose();

  // Take care when changing this value - if too high, it could get restricted
  // by the max speed of the tracks.
  const double rotationSpeed = 0.25;
  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(0, 0, 0));
  msgs::Set(msg.mutable_angular(),
    ignition::math::Vector3d(0, 0, rotationSpeed));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(), middlePose.Pos().X(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), middlePose.Pos().Y(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), middlePose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), middlePose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), middlePose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              middlePose.Rot().Yaw() - rotationSpeed,
              1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test following a circular path.

  // This also tests the fix from commit b1836a3 still works. If not, the test
  // will segfault in debug mode, because there was a use-after-free problem.

  const auto lastPose = model->WorldPose();

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0.2));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(), lastPose.Pos().X() + 0.4, 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), lastPose.Pos().Y() - 0.15, 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), lastPose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), lastPose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), lastPose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              lastPose.Rot().Yaw() - 0.37,
              1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test driving on staircase - should climb to its middle part.

  this->world->Reset();

  const auto beforeStairsPose = ignition::math::Pose3d(
      0, 3, 0.1,
      0, 0, -ignition::math::Angle::HalfPi.Radian());
  model->SetWorldPose(beforeStairsPose);

  // Let the model settle down.
  this->world->Step(300);

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(3500);

  EXPECT_NEAR(model->WorldPose().Pos().X(), 0.0, 0.15);
  EXPECT_LE(model->WorldPose().Pos().Y(), 0.7);
  EXPECT_GT(model->WorldPose().Pos().Z(), 0.6);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), -0.4, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              beforeStairsPose.Rot().Yaw(), 1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test driving over a cylinder - this is a check of the bugfix
  // released in commit 6196a2a. In some cases, the track isn't the "collision
  // body nr. 1", but it is nr. 2. The contact normal is then flipped and the
  // computed friction direction is therefore wrong. This situation isn't easy
  // to reach, ie. the track is the first collision body in most cases. But it
  // seems the cylinder likes to be first even more :)

  this->world->Reset();

  const auto beforeCylinderPose = ignition::math::Pose3d(
    0, -2, 0.1,
    0, 0, -ignition::math::Angle::HalfPi.Radian());
  model->SetWorldPose(beforeCylinderPose);

  // Let the model settle down.
  this->world->Step(300);

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(2000);

  // The cylinder is at (0, -3, 0), we start at (0, -2, 0), and want to pass
  // at least a bit behind the cylinder (0, -3.5, 0). The driving is a bit wild,
  // so we don't care much about the end X position and yaw.
  EXPECT_NEAR(model->WorldPose().Pos().X(), 0.0, 0.5);  // The driving is wild
  EXPECT_LT(model->WorldPose().Pos().Y(), -3.5);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              beforeStairsPose.Rot().Yaw(), 0.5);  // The driving is wild

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test driving over an obstacle that requires flippers. Without them, the
  // robot would get stuck in front of the obstacle.

  this->world->Reset();

  const auto beforeBoxPose = ignition::math::Pose3d(
      2, -0.5, 0.1,
      0, 0, ignition::math::Angle::HalfPi.Radian());
  model->SetWorldPose(beforeBoxPose);

  // Let the model settle down.
  this->world->Step(300);

  // we go backwards because we have the CoG in the back
  msgs::Set(msg.mutable_linear(),
            ignition::math::Vector3d(-forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(4000);

  // The box is at (2, -2, 0), we start at (2, -0.5, 0), and want to pass
  // at least a bit behind the box (2, -3, 0). The driving is a bit wild.
  EXPECT_NEAR(model->WorldPose().Pos().X(), 2.0, 0.1);  // The driving is wild
  EXPECT_LT(model->WorldPose().Pos().Y(), -3);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              beforeBoxPose.Rot().Yaw(), 0.25);  // The driving is wild
  // And we go back, which is a somewhat easier way

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(4000);

  // We start at (2, -0.5, 0), we go back, and it should be a bit faster than
  // the previous traversal, so we should end up beyond the starting point.
  EXPECT_NEAR(model->WorldPose().Pos().X(), 2.0, 0.1);  // The driving is wild
  EXPECT_GT(model->WorldPose().Pos().Y(), -0.5);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              beforeBoxPose.Rot().Yaw(), 0.25);  // The driving is wild

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);
}

//// Test that the WheelTracked vehicle is moving as expected.
TEST_F(WheeledTrackedVehiclesTest, WheelTracked)
{
  ASSERT_NE(this->world->ModelCount(), 0u);
  physics::ModelPtr model = this->world->ModelByName("wheel_tracked");
  ASSERT_NE(model, nullptr);
  ASSERT_EQ(model->GetName(), "wheel_tracked");

  auto modelClone = this->world->ModelByName("wheel_tracked_clone");
  ASSERT_NE(modelClone, nullptr);

  transport::PublisherPtr pub = node->Advertise<msgs::Twist>(
    "~/wheel_tracked/cmd_vel_twist");
  pub->WaitForConnection();

  // Move the robot somewhere to free space without obstacles.
  model->SetWorldPose(ignition::math::Pose3d(10, 10, 0.1, 0, 0, 0));

  // Let the models settle down.
  this->world->Step(300);

  // Test straight driving - 1 sec driving, should move 1 meter forward.

  const auto startPose = model->WorldPose();
  const auto startPoseClone = modelClone->WorldPose();

  const double forwardSpeed = 1.0;
  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(),
              startPose.Pos().X() + forwardSpeed,
              0.2);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), startPose.Pos().Y(), 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), startPose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), startPose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), startPose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(), startPose.Rot().Yaw(), 1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test rotation in place - 1 sec rotation, should turn 0.25 rad.

  const auto middlePose = model->WorldPose();

  // Take care when changing this value - if too high, it could get restricted
  // by the max speed of the tracks.
  const double rotationSpeed = 0.25;
  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(0, 0, 0));
  msgs::Set(msg.mutable_angular(),
    ignition::math::Vector3d(0, 0, rotationSpeed));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(), middlePose.Pos().X(), 0.2);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), middlePose.Pos().Y(), 0.2);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), middlePose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), middlePose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), middlePose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              middlePose.Rot().Yaw() - rotationSpeed,
              0.2);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test following a circular path.

  const auto lastPose = model->WorldPose();

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0.2));
  pub->Publish(msg, true);
  this->world->Step(1000);

  EXPECT_NEAR(model->WorldPose().Pos().X(), lastPose.Pos().X() + 0.4, 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), lastPose.Pos().Y() - 0.15, 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), lastPose.Pos().Z(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), lastPose.Rot().Roll(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), lastPose.Rot().Pitch(), 1e-2);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              lastPose.Rot().Yaw() - 0.37,
              0.4);  // TODO lower to 0.2 when directional friction is fixed

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);

  // Test driving on staircase - should climb to its front part.

  this->world->Reset();

  const auto beforeStairsPose = ignition::math::Pose3d(
    0, 3, 0.1,
    0, 0, -ignition::math::Angle::HalfPi.Radian());
  model->SetWorldPose(beforeStairsPose);

  // Let the model settle down.
  this->world->Step(300);

  msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(forwardSpeed, 0, 0));
  msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, 0));
  pub->Publish(msg, true);
  this->world->Step(3500);

  // This model gets stuck on the first steps.
  EXPECT_NEAR(model->WorldPose().Pos().X(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Pos().Y(), 1.6, 0.4);
  EXPECT_NEAR(model->WorldPose().Pos().Z(), 0.2, 0.2);
  EXPECT_NEAR(model->WorldPose().Rot().Roll(), 0.0, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Pitch(), -0.4, 1e-1);
  EXPECT_NEAR(model->WorldPose().Rot().Yaw(),
              beforeStairsPose.Rot().Yaw(), 1e-1);

  // The cloned model should not move
  verifyPose(modelClone, startPoseClone);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
