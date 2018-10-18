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

#include <ignition/math/Pose3.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;

class Issue2239Test: public JointTest
{
  /// \brief Spawn a revolute2 joint and test whether the model teleports
  public: void TestRevolute2JointTeleport(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void Issue2239Test::TestRevolute2JointTeleport(
    const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(nullptr, world);

  // Verify the physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_NE(nullptr, physics);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(ignition::math::Vector3d::Zero);

  SpawnJointOptions opt;
  opt.type = "revolute2";
  opt.parentLinkPose.Set(0, 0.1, 0.1, 0, 0, 0);
  opt.childLinkPose.Set(0.1, 0.2, 0.1, 0, 0, 0);
  opt.axis.Set(1, 0, 0);
  opt.wait = common::Time(99, 0);
  physics::JointPtr joint = SpawnJoint(opt);
  ASSERT_NE(nullptr, joint);

  physics::LinkPtr childLink = joint->GetChild();
  ASSERT_NE(nullptr, childLink);

  world->Step(1);

  const auto childPos = childLink->WorldCoGPose().Pos();
  const double tolerance = 1e-4;
  // We expect the child link to be in the same position as it was when it was
  // created. The regression is that the child link teleports to the origin.
  EXPECT_NEAR(childPos.X(), opt.childLinkPose.Pos().X(), tolerance);
  EXPECT_NEAR(childPos.Y(), opt.childLinkPose.Pos().Y(), tolerance);
  EXPECT_NEAR(childPos.Z(), opt.childLinkPose.Pos().Z(), tolerance);
}

TEST_P(Issue2239Test, TestRevolute2JointTeleport)
{
  this->TestRevolute2JointTeleport(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue2239Test,
  ::testing::Combine(::testing::Values("ode", "dart"), ::testing::Values("")));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
