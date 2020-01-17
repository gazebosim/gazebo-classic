/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class VariableGearboxTest : public ServerFixture
{
};

TEST_F(VariableGearboxTest, DemoJointTypes)
{
  Load("worlds/variable_gearbox_plugin.world", true, "ode");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::ModelPtr model = world->ModelByName("parent");
  ASSERT_NE(model, nullptr);

  const std::string prefix = "demo_joint_types::gearbox_";
  physics::JointPtr gearbox = model->GetJoint(prefix + "demo");
  ASSERT_NE(gearbox, nullptr);
  ASSERT_TRUE(gearbox->HasType(physics::Base::GEARBOX_JOINT));

  physics::JointPtr inputJoint = model->GetJoint(prefix + "input_joint");
  ASSERT_NE(inputJoint, nullptr);
  physics::JointPtr outputJoint = model->GetJoint(prefix + "output_joint");
  ASSERT_NE(outputJoint, nullptr);

  // Confirm intput and output are initially at rest
  EXPECT_DOUBLE_EQ(0.0, inputJoint->GetVelocity(0));
  EXPECT_DOUBLE_EQ(0.0, outputJoint->GetVelocity(0));

  // Initial gearbox parameters from model file
  EXPECT_DOUBLE_EQ(5.0, gearbox->GetParam("ratio", 0));
  EXPECT_DOUBLE_EQ(0.0, gearbox->GetParam("reference_angle1", 0));
  EXPECT_DOUBLE_EQ(0.0, gearbox->GetParam("reference_angle2", 0));

  // Take 1 step so the plugin can update ratio and reference angles
  world->Step(1);
  // for some reason the sign of the gear ratio sign is inverted
  EXPECT_DOUBLE_EQ(1.0, gearbox->GetParam("ratio", 0));
  EXPECT_DOUBLE_EQ(0.0, gearbox->GetParam("reference_angle1", 0));
  EXPECT_DOUBLE_EQ(0.0, gearbox->GetParam("reference_angle2", 0));

  // Step forward to high-gear
  world->Step(500);
  // Pendulum connected to input joint should swing past the gear ratio change
  // so that the gear ratio is higher
  EXPECT_DOUBLE_EQ(20.0, gearbox->GetParam("ratio", 0));
  EXPECT_NEAR(inputJoint->GetVelocity(0), 2.34, 1e-2);
  EXPECT_NEAR(outputJoint->GetVelocity(0), -46.8, 1e-1);
  EXPECT_NEAR(-20*inputJoint->GetVelocity(0),
                 outputJoint->GetVelocity(0), 2e-3);
  EXPECT_DOUBLE_EQ(gearbox->GetParam("reference_angle1", 0) - (-7.5),
            -20 * (gearbox->GetParam("reference_angle2", 0) - 1.8));

  // Step back to low-gear
  world->Step(1800);
  EXPECT_DOUBLE_EQ(1.0, gearbox->GetParam("ratio", 0));
  EXPECT_NEAR(inputJoint->GetVelocity(0), -9.79, 1e-2);
  EXPECT_NEAR(outputJoint->GetVelocity(0), 9.79, 5e-3);
  EXPECT_NEAR(-inputJoint->GetVelocity(0), outputJoint->GetVelocity(0), 2e-2);
  EXPECT_DOUBLE_EQ(gearbox->GetParam("reference_angle1", 0),
                  -gearbox->GetParam("reference_angle2", 0));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
