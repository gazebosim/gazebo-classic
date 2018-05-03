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

#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class ContainPluginTest : public ServerFixture
{
};

// Flag turned to true once box contains.
bool g_contain = false;

//////////////////////////////////////////////////
// Callback for box/contains topic
void boxCb(const ignition::msgs::Boolean &_msg)
{
  g_contain = _msg.data();
}

//////////////////////////////////////////////////
TEST_F(ContainPluginTest, Disable)
{
  this->Load("worlds/contain_plugin_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get models
  auto drill = world->ModelByName("drill");
  ASSERT_NE(nullptr, drill);

  // Subscribe to plugin notifications
  std::string prefix("/gazebo/default/drill/");

  ignition::transport::Node ignNode;
  ignNode.Subscribe(prefix + "contain", &boxCb);

  // Check box doesn't contain yet
  EXPECT_FALSE(g_contain);

  // Place drill inside box
  drill->SetWorldPose(ignition::math::Pose3d(10.0, 10.0, 1.0, 0, 0, 0));

  // Give it time to fall
  world->Step(1000);

  // Verify we get a notification
  EXPECT_TRUE(g_contain);

  // Place drill outside box
  drill->SetWorldPose(ignition::math::Pose3d(0.0, 0.0, 1.0, 0, 0, 0));

  // Give it time to fall
  world->Step(1000);

  // Verify we get a notification
  EXPECT_FALSE(g_contain);

  // Disable plugin
  unsigned int timeout = 300u;
  ignition::msgs::Boolean req;
  ignition::msgs::Boolean rep;
  bool result;

  {
    req.set_data(false);

    auto executed = ignNode.Request(prefix + "enable", req, timeout, rep,
        result);

    EXPECT_TRUE(executed);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // Place drill inside box
  drill->SetWorldPose(ignition::math::Pose3d(10.0, 10.0, 1.0, 0, 0, 0));

  // Give it time to fall
  world->Step(1000);

  // Wait and see it doesn't notify now
  EXPECT_FALSE(g_contain);

  // Fail to re-disable plugin
  {
    req.set_data(false);

    auto executed = ignNode.Request(prefix + "enable", req, timeout, rep,
        result);

    EXPECT_TRUE(executed);
    EXPECT_FALSE(result);
    EXPECT_FALSE(rep.data());
  }

  // Re-enable plugin
  {
    req.set_data(true);

    auto executed = ignNode.Request(prefix + "enable", req, timeout, rep,
        result);

    EXPECT_TRUE(executed);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());
  }

  // Wait and see we receive a notification with the latest status
  world->Step(1);
  EXPECT_TRUE(g_contain);

  // Fail to re-enable plugin
  {
    req.set_data(true);

    auto executed = ignNode.Request(prefix + "enable", req, timeout, rep,
        result);

    EXPECT_TRUE(executed);
    EXPECT_FALSE(result);
    EXPECT_FALSE(rep.data());
  }
}

//////////////////////////////////////////////////
TEST_F(ContainPluginTest, MovingGeometry)
{
  this->Load("worlds/contain_plugin_moving_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Subscribe to plugin notifications
  std::string prefix("/gazebo/default/drill/");

  ignition::transport::Node ignNode;
  ignNode.Subscribe(prefix + "contain", &boxCb);

  // Box initially does not contain drill
  world->Step(10);
  EXPECT_FALSE(g_contain);

  // Box contains drill after box falls for a bit
  world->Step(400);
  EXPECT_TRUE(g_contain);

  // Box doesn't contain drill after falling a bit more
  world->Step(400);
  EXPECT_FALSE(g_contain);
}

//////////////////////////////////////////////////
TEST_F(ContainPluginTest, RemoveEntity)
{
  this->Load("worlds/contain_plugin_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Get models
  auto drill = world->ModelByName("drill");
  ASSERT_NE(nullptr, drill);

  // Subscribe to plugin notifications
  std::string prefix("/gazebo/default/drill/");

  ignition::transport::Node ignNode;
  ignNode.Subscribe(prefix + "contain", &boxCb);

  // Place drill inside box
  drill->SetWorldPose(ignition::math::Pose3d(10.0, 10.0, 1.0, 0, 0, 0));
  world->Step(10);
  EXPECT_TRUE(g_contain);

  // Delete drill
  world->RemoveModel(drill);
  drill = nullptr;

  // Box doesn't contain drill since drill does not exist
  world->Step(10);
  EXPECT_FALSE(g_contain);
}

//////////////////////////////////////////////////
TEST_F(ContainPluginTest, RemoveReferenceEntity)
{
  this->Load("worlds/contain_plugin_moving_demo.world", true);
  auto world = physics::get_world();
  ASSERT_NE(nullptr, world);

  // Subscribe to plugin notifications
  std::string prefix("/gazebo/default/drill/");

  ignition::transport::Node ignNode;
  ignNode.Subscribe(prefix + "contain", &boxCb);

  // Box contains drill after box falls for a bit
  world->Step(400);
  EXPECT_TRUE(g_contain);

  // Delete box
  auto box = world->ModelByName("box");
  ASSERT_NE(nullptr, box);
  world->RemoveModel(box);
  box = nullptr;

  // No box, so nothing contains the drill
  world->Step(10);
  EXPECT_FALSE(g_contain);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
