/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <string>
#include <type_traits>

#include <ignition/math/Pose3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class MisalignmentPluginTest : public ServerFixture
{
  /// \brief gtest calls this before running a test
  protected: virtual void SetUp() override
  {
    this->ServerFixture::SetUp();
    this->lastPoseMsg = nullptr;
  }

  public: void MisalignmentCallback(ConstPoseStampedPtr &_msg)
  {
    lastPoseMsg.reset(new msgs::PoseStamped(*_msg));
  }

  /// \brief last pose message received
  public: msgs::PoseStampedPtr lastPoseMsg;
};

//////////////////////////////////////////////////
TEST_F(MisalignmentPluginTest, Disable)
{
  msgs::Int enableMsg;

  this->Load("worlds/misalignment_plugin_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  std::string firstPrefix = "/gazebo/default/drill/alignment_1";

  ASSERT_EQ(nullptr, this->lastPoseMsg);

  // Subscribe to plugin notifications
  auto subscriber = this->node->Subscribe(firstPrefix + "/misalignment",
        &MisalignmentPluginTest::MisalignmentCallback,
        dynamic_cast<MisalignmentPluginTest*>(this));
  ASSERT_NE(subscriber, nullptr);

  // make sure a message is received
  world->Step(10);
  ASSERT_NE(nullptr, this->lastPoseMsg.get());

  // Disable plugin
  auto enablePub = this->node->Advertise<msgs::Int>(firstPrefix + "/enable");
  enableMsg.set_data(0);
  enablePub->Publish(enableMsg);

  // Make sure no message is received
  world->Step(10);
  this->lastPoseMsg = nullptr;
  world->Step(100);
  ASSERT_EQ(nullptr, this->lastPoseMsg);

  // Enable plugin again
  enableMsg.set_data(1);
  enablePub->Publish(enableMsg);

  // Make sure a message is received
  world->Step(10);
  ASSERT_NE(nullptr, this->lastPoseMsg.get());
}

//////////////////////////////////////////////////
TEST_F(MisalignmentPluginTest, CompareAlignments)
{
  msgs::Int enableMsg;

  this->Load("worlds/misalignment_plugin_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  std::string firstPrefix = "/gazebo/default/drill/alignment_1";
  std::string secondPrefix = "/gazebo/default/drill/alignment_2";

  ASSERT_EQ(nullptr, this->lastPoseMsg);

  ignition::math::Pose3d firstPose;
  ignition::math::Pose3d secondPose;

  // Get a message from the first plugin
  {
    auto subscriber = this->node->Subscribe(firstPrefix + "/misalignment",
          &MisalignmentPluginTest::MisalignmentCallback,
          dynamic_cast<MisalignmentPluginTest*>(this));
    ASSERT_NE(subscriber, nullptr);

    // make sure a message is received
    world->Step(10);
    ASSERT_NE(nullptr, this->lastPoseMsg.get());
    firstPose = msgs::ConvertIgn(this->lastPoseMsg->pose());
  }

  // clear any data from first subscriber
  world->Step(10);
  this->lastPoseMsg = nullptr;

  // Get a message from the second plugin
  {
    auto subscriber = this->node->Subscribe(secondPrefix + "/misalignment",
          &MisalignmentPluginTest::MisalignmentCallback,
          dynamic_cast<MisalignmentPluginTest*>(this));
    ASSERT_NE(subscriber, nullptr);

    // make sure a message is received
    world->Step(10);
    ASSERT_NE(nullptr, this->lastPoseMsg.get());
    secondPose = msgs::ConvertIgn(this->lastPoseMsg->pose());
  }

  EXPECT_EQ(firstPose, secondPose);
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
