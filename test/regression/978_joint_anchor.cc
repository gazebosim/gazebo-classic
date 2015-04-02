/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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

#include "ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;

class Issue978Test : public JointTest
{
  public: void JointAnchor(const std::string &_physicsEngine);
};


/////////////////////////////////////////////////
// \brief Test for issue #978
void Issue978Test::JointAnchor(const std::string &_physicsEngine)
{
  // Abort test for simbody, since SimbodyJoint::GetAnchor isn't implemented
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #979.\n";
    return;
  }

  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::string _jointType = "revolute";
  {
    gzdbg << "SpawnJoint " << _jointType << " child parent" << std::endl;
    SpawnJointOptions opt;
    opt.type = _jointType;
    opt.worldChild = false;
    opt.worldParent = false;
    opt.noLinkPose = true;
    opt.modelPose = math::Pose(1, 2, 3, 0, 0, 0);

    physics::JointPtr joint = SpawnJoint(opt);
    ASSERT_TRUE(joint != NULL);

    // Check child and parent links
    physics::LinkPtr child = joint->GetChild();
    physics::LinkPtr parent = joint->GetParent();
    ASSERT_TRUE(child != NULL);
    EXPECT_EQ(child->GetParentJoints().size(), 1u);
    EXPECT_EQ(child->GetChildJoints().size(), 0u);
    ASSERT_TRUE(parent != NULL);
    EXPECT_EQ(parent->GetChildJoints().size(), 1u);
    EXPECT_EQ(parent->GetParentJoints().size(), 0u);

    // Check anchor location
    EXPECT_EQ(joint->GetAnchor(0), opt.modelPose.pos);
  }
}

TEST_P(Issue978Test, JointAnchor)
{
  JointAnchor(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue978Test,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("")));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
