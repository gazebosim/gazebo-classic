/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class TouchPluginTest : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  /// \brief Test observing only one link in a model
  /// \param[in] _physicsEngine Physics engine to use.
  public: void OneLink(const std::string &_physicsEngine);

  /// \brief Test observing more than one link in a model
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MultiLink(const std::string &_physicsEngine);

  /// \brief Test a plugin which starts disabled.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void StartDisabled(const std::string &_physicsEngine);

  /// \brief Callback when white box plugin touches.
  /// \param[in] _msg Unused
  public: void WhiteTouch(ConstIntPtr &/*_msg*/);

  /// \brief Callback when red / yellow box plugin touches.
  /// \param[in] _msg Unused
  public: void RedTouch(ConstIntPtr &/*_msg*/);

  /// \brief Callback when blue box plugin touches.
  /// \param[in] _msg Unused
  public: void BlueTouch(ConstIntPtr &/*_msg*/);

  /// \brief Flag turned to true once white touches.
  private: bool whiteTouched = false;

  /// \brief Flag turned to true once red/yellow touches.
  private: bool redTouched = false;

  /// \brief Flag turned to true once blue touches.
  private: bool blueTouched = false;
};

//////////////////////////////////////////////////
void TouchPluginTest::WhiteTouch(ConstIntPtr &/*_msg*/)
{
  this->whiteTouched = true;
}

//////////////////////////////////////////////////
void TouchPluginTest::RedTouch(ConstIntPtr &/*_msg*/)
{
  this->redTouched = true;
}

//////////////////////////////////////////////////
void TouchPluginTest::BlueTouch(ConstIntPtr &/*_msg*/)
{
  this->blueTouched = true;
}

//////////////////////////////////////////////////
void TouchPluginTest::OneLink(const std::string &_physicsEngine)
{
  this->Load("worlds/touch_plugin_demo.world", true, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  // Get models
  auto whiteBox = world->ModelByName("white_box");

  // Subscribe to plugin notifications
  auto whiteSub = this->node->Subscribe("/white_touches_only_green/touched",
      &TouchPluginTest::WhiteTouch, this);

  // Check boxes haven't touched yet
  EXPECT_FALSE(this->whiteTouched);

  // Place white box on top of green box
  whiteBox->SetWorldPose(ignition::math::Pose3d(4.0, 0, 2.0, 0, 0, 0));

  // Give it time to drop and touch
  world->Step(1000);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(this->whiteTouched);

  // Give it time to touch for 3 seconds
  world->Step(4000);

  // Check it has touched for long enough
  EXPECT_TRUE(this->whiteTouched);

  // Wait more and check it doesn't notify again
  this->whiteTouched = false;
  world->Step(5000);
  EXPECT_FALSE(this->whiteTouched);

  // Enable plugin again
  auto enablePub = this->node->Advertise<msgs::Int>(
        "/white_touches_only_green/enable");

  msgs::Int msg;
  msg.set_data(1);
  enablePub->Publish(msg);

  // Wait and see it notifies again
  this->whiteTouched = false;
  world->Step(5000);
  EXPECT_TRUE(this->whiteTouched);
}

//////////////////////////////////////////////////
TEST_P(TouchPluginTest, OneLink)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine != "ode")
  {
    gzwarn << "This test depends on continuous contacts, which are achieved on"
           << " ODE using <max_vel> and <min_depth>. Skipping test for ["
           << physicsEngine << "]. See issue #2213." << std::endl;
    return;
  }
  OneLink(GetParam());
}

//////////////////////////////////////////////////
void TouchPluginTest::MultiLink(const std::string &_physicsEngine)
{
  this->Load("worlds/touch_plugin_demo.world", true, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  // Get models
  auto redYellowBox = world->ModelByName("red_yellow_box");

  // Subscribe to plugin notifications
  auto redSub = this->node->Subscribe(
        "/red_and_yellow_touch_only_green/touched",
        &TouchPluginTest::RedTouch, this);

  // Check boxes haven't touched yet
  EXPECT_FALSE(this->redTouched);

  // Place red and yellow boxes on top of green box
  redYellowBox->SetWorldPose(
      ignition::math::Pose3d(4.0, 0, 2.0, IGN_PI*0.5, 0, 0));

  // Give it time to drop and touch
  world->Step(1000);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(this->redTouched);

  // Give it time to touch for 5 seconds
  world->Step(5000);

  // Check it has touched for long enough
  EXPECT_TRUE(this->redTouched);
}

//////////////////////////////////////////////////
TEST_P(TouchPluginTest, MultiLink)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine != "ode")
  {
    gzwarn << "This test depends on continuous contacts, which are achieved on"
           << " ODE using <max_vel> and <min_depth>. Skipping test for ["
           << physicsEngine << "]. See issue #2213." << std::endl;
    return;
  }
  MultiLink(GetParam());
}

//////////////////////////////////////////////////
void TouchPluginTest::StartDisabled(const std::string &_physicsEngine)
{
  this->Load("worlds/touch_plugin_demo.world", true, _physicsEngine);
  auto world = physics::get_world();
  ASSERT_NE(world , nullptr);

  // Get models
  auto blueBox = world->ModelByName("blue_box");

  // Subscribe to plugin notifications
  auto blueSub = this->node->Subscribe("/blue_touches_only_green/touched",
        &TouchPluginTest::BlueTouch, this);

  // Check boxes haven't touched yet
  EXPECT_FALSE(this->blueTouched);

  // Place blue box on top of green box
  blueBox->SetWorldPose(ignition::math::Pose3d(4.0, 0, 2.0, 0, 0, 0));

  // Give it time to drop and touch for a while
  world->Step(3000);

  // Verify we don't get a notification
  EXPECT_FALSE(this->blueTouched);

  // Enable plugin
  auto enablePub = this->node->Advertise<msgs::Int>(
        "/blue_touches_only_green/enable");

  msgs::Int msg;
  msg.set_data(1);
  enablePub->Publish(msg);

  // Wait and see it notifies now
  this->blueTouched = false;
  world->Step(3000);
  EXPECT_TRUE(this->blueTouched);
}

//////////////////////////////////////////////////
TEST_P(TouchPluginTest, StartDisabled)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine != "ode")
  {
    gzwarn << "This test depends on continuous contacts, which are achieved on"
           << " ODE using <max_vel> and <min_depth>. Skipping test for ["
           << physicsEngine << "]. See issue #2213." << std::endl;
    return;
  }
  StartDisabled(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, TouchPluginTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
