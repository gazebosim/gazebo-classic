/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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

class Issue494Test : public JointTest
{
  public: void CheckAxisFrame(const std::string &_physicsEngine,
                              const std::string &_jointType);
};


/////////////////////////////////////////////////
// \brief Test for issue #494
void Issue494Test::CheckAxisFrame(const std::string &_physicsEngine,
                                  const std::string &_jointType)
{
  if (SDF_MAJOR_VERSION == 1 && SDF_MINOR_VERSION < 5)
  {
    gzerr << "SDF version [" << SDF_VERSION << "] detected, "
          << "1.5 or greater required"
          << std::endl;
    return;
  }
  if (!((_physicsEngine == "ode" || _physicsEngine == "bullet")
      && _jointType == "revolute"))
  {
    gzerr << "This test doesn't yet work for [" << _physicsEngine
          << "] with joint type [" << _jointType << "]"
          << std::endl;
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

  SpawnJointOptions opt;
  opt.type = _jointType;
  double Am = M_PI / 12;
  double Al = M_PI / 12;
  double Aj = M_PI / 12;
  opt.modelPose.rot.SetFromEuler(0, 0, Am);
  opt.childLinkPose.rot.SetFromEuler(0, 0, Al);
  opt.jointPose.rot.SetFromEuler(0, 0, Aj);
  opt.axis.Set(1, 0, 0);

  // i = 0: child parent
  // i = 1: child world 
  // i = 2: world parent
  for (int i = 0; i < 3; ++i)
  {
    gzdbg << "SpawnJoint " << _jointType;
    if (i / 2)
    {
      opt.worldChild = true;
      std::cout << " world";
    }
    else
    {
      opt.worldChild = false;
      std::cout << " child";
    }
    if (i % 2)
    {
      opt.worldParent = true;
      std::cout << " world";
    }
    else
    {
      opt.worldParent = false;
      std::cout << " parent";
    }
    std::cout << std::endl;

    // parent model frame
    {
      opt.useParentModelFrame = true;
      physics::JointPtr jointUseParentModelFrame = SpawnJoint(opt);
      ASSERT_TRUE(jointUseParentModelFrame != NULL);

      if (opt.worldParent)
      {
        EXPECT_EQ(opt.axis, jointUseParentModelFrame->GetGlobalAxis(0));
      }
      else
      {
        math::Vector3 referenceAxis(cos(Am), sin(Am), 0);
        EXPECT_EQ(referenceAxis, jointUseParentModelFrame->GetGlobalAxis(0));
      }
    }

    // joint frame
    {
      opt.useParentModelFrame = false;
      physics::JointPtr joint = SpawnJoint(opt);
      ASSERT_TRUE(joint != NULL);

      if (opt.worldChild)
      {
        math::Vector3 referenceAxis(cos(Aj), sin(Aj), 0);
        EXPECT_EQ(referenceAxis, joint->GetGlobalAxis(0));
      }
      else
      {
        math::Vector3 referenceAxis(cos(Am+Al+Aj), sin(Am+Al+Aj), 0);
        EXPECT_EQ(referenceAxis, joint->GetGlobalAxis(0));
      }
    }
  }
}

TEST_P(Issue494Test, CheckAxisFrame)
{
  CheckAxisFrame(this->physicsEngine, this->jointType);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue494Test,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute", "prismatic")));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
