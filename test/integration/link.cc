/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <boost/algorithm/string/replace.hpp>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class LinkTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// This tests wrench subscriber and static links
TEST_F(LinkTest, StaticWrench)
{
  this->Load("test/worlds/static.world");

  // Get models
  auto model0 = this->GetModel("model_0");
  EXPECT_TRUE(model0->IsStatic());

  auto link0 = model0->GetLink("link");
  EXPECT_TRUE(link0->IsStatic());

  ignition::math::Pose3d model0Initial(1, 0, 0.5, 0, 0, 0);
  EXPECT_EQ(model0->GetWorldPose(), model0Initial);

  auto model1 = this->GetModel("model_1");
  EXPECT_FALSE(model1->IsStatic());

  auto link1 = model1->GetLink("link");
  EXPECT_FALSE(link1->IsStatic());

  ignition::math::Pose3d model1Initial(-1, 0, 0.5, 0, 0, 0);
  EXPECT_EQ(model1->GetWorldPose(), model1Initial);

  // Setup wrench publishers
  std::string topicName = "~/";
  topicName += link0->GetScopedName() + "/wrench";
  boost::replace_all(topicName, "::", "/");

  auto wrenchModel0Pub = this->node->Advertise<msgs::Wrench>(topicName);

  topicName = "~/";
  topicName += link1->GetScopedName() + "/wrench";
  boost::replace_all(topicName, "::", "/");

  auto wrenchModel1Pub = this->node->Advertise<msgs::Wrench>(topicName);

  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), ignition::math::Vector3d(0, 10000, 0));
  msgs::Set(msg.mutable_torque(), ignition::math::Vector3d::Zero);

  // Send wrench msg to non static model and check it moves
  wrenchModel1Pub->Publish(msg);

  int sleep = 0;
  int maxSleep = 30;
  while (model1->GetWorldPose() == model1Initial && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_NE(model1->GetWorldPose(), model1Initial);

  // Send wrench msg to static model and check it doesn't move
  wrenchModel0Pub->Publish(msg);

  maxSleep = sleep + 5;
  sleep = 0;
  while (model0->GetWorldPose() == model0Initial && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(model0->GetWorldPose(), model0Initial);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
