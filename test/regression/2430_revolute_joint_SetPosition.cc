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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

/////////////////////////////////////////////////
class Issue2430Test :
    public ServerFixture,
    public ::testing::WithParamInterface<std::string>
{
  /////////////////////////////////////////////////
  public: virtual void SetUp() override
  {
    const ::testing::TestInfo *const test_info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    if (test_info->value_param())
    {
      gzdbg << "Params: " << test_info->value_param() << std::endl;
      this->physicsEngine = GetParam();
    }
  }

  /////////////////////////////////////////////////
  public: void TestJointInitialization(const double initialPosition)
  {
    if(   "simbody" == this->physicsEngine
       || "dart" == this->physicsEngine)
    {
      gzdbg << "Test is disabled for " << this->physicsEngine << ", skipping\n";
      // This test is disabled for Simbody and DART, because the ability to set
      // joint positions for those physics engines is not available yet in
      // Gazebo.
      return;
    }

    gzdbg << "Testing [" << initialPosition*180.0/M_PI << "] degrees\n";

    // Load the test world
    this->Load("worlds/test/issue_2430_revolute_joint_SetPosition.world",
               true, this->physicsEngine, {});
    physics::WorldPtr world = physics::get_world("default");
    ASSERT_NE(nullptr, world);

    // Verify the physics engine
    physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    ASSERT_NE(nullptr, physics);

    physics::ModelPtr model = world->GetModel("revolute_model");
    ASSERT_NE(nullptr, model);

    physics::JointPtr joint = model->GetJoint("revolute_joint");
    ASSERT_NE(nullptr, joint);

    EXPECT_NEAR(0.0, joint->GetAngle(0).Radian(), 1e-6);

    joint->SetPosition(0, initialPosition);

    world->Step(1);

    // There is no gravity, and there are no forces acting on the bodies or the
    // joint, so the joint position value should still be the same.

    const double resultPosition = joint->GetAngle(0).Radian();

    EXPECT_NEAR(initialPosition, resultPosition, 1e-6);
  }

  protected: std::string physicsEngine;
};

/////////////////////////////////////////////////
TEST_P(Issue2430Test, Positive20Degrees)
{
  this->TestJointInitialization( 20.0*M_PI/180.0);
}

/////////////////////////////////////////////////
TEST_P(Issue2430Test, Negative20Degrees)
{
  this->TestJointInitialization(-20.0*M_PI/180.0);
}

/////////////////////////////////////////////////
TEST_P(Issue2430Test, Positive200Degrees)
{
  this->TestJointInitialization( 200.0*M_PI/180.0);
}

/////////////////////////////////////////////////
TEST_P(Issue2430Test, Negative200Degrees)
{
  this->TestJointInitialization(-200.0*M_PI/180.0);
}


/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue2430Test, PHYSICS_ENGINE_VALUES);


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
