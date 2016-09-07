/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/physics/Joint.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

#define TOL 1e-6

using namespace gazebo;

class JointAnchorTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Spawn model with each type of joint.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void CheckJointAnchor(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// Test for spawning each joint type
void JointAnchorTest::CheckJointAnchor(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "simbody Joint::GetAnchor not yet implemented.\n";
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "dart Joint::GetAnchor needs attention, causes segfault.\n";
    return;
  }

  // Load an empty world
  Load("worlds/joint_anchor.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  physics::ModelPtr model = world->GetModel("model_2");
  physics::JointPtr joint = model->GetJoint("joint_01");

  ignition::math::Vector3d anchor = joint->GetAnchor(0).Ign();
  gzdbg << anchor << "\n";
  EXPECT_NEAR(anchor.X(), 0, TOL);
  EXPECT_NEAR(anchor.Y(), 0, TOL);
  EXPECT_NEAR(anchor.Z(), 1.1, TOL);
}

TEST_P(JointAnchorTest, CheckJointAnchor)
{
  CheckJointAnchor(GetParam());
}

INSTANTIATE_TEST_CASE_P(TestRuns, JointAnchorTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

