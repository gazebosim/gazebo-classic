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

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
const double g_tolerance = 1e-2;

class JointControlPluginTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(JointControlPluginTest, InitializeJointControllers)
{
  Load("worlds/init_joint_control.world", true);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  physics::ModelPtr model = world->ModelByName("double_pendulum_with_base");
  ASSERT_TRUE(model != nullptr);

  physics::JointPtr upperJoint = model->GetJoint("upper_joint");
  ASSERT_TRUE(upperJoint != nullptr);
  physics::JointPtr lowerJoint = model->GetJoint("lower_joint");
  ASSERT_TRUE(lowerJoint != nullptr);

  world->Step(4000);

  EXPECT_NEAR(upperJoint->Position(0), -1.5708, g_tolerance);
  EXPECT_NEAR(upperJoint->GetVelocity(0), 0.0, g_tolerance);

  EXPECT_NEAR(lowerJoint->Position(0), -2.7124, g_tolerance);
  EXPECT_NEAR(lowerJoint->GetVelocity(0), 0.0, g_tolerance);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
