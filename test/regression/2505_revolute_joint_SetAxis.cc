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
#include "../../util.hh"

using namespace gazebo;

/////////////////////////////////////////////////
class Issue2505Test
    : public ServerFixture,
      public ::testing::WithParamInterface<const char *>
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
  public: void TestAxis()
  {
    if ( "simbody" == this->physicsEngine )
    {
      gzdbg << "Test is disabled for " << this->physicsEngine << ", skipping\n";
      // This test is disabled for Simbody, because the ability to set
      // joint positions for that physics engine is not available yet in
      // Gazebo.
      return;
    }

    // Load the test world
    this->Load("worlds/test/issue_2505_revolute_joint_SetAxis.world",
               true, this->physicsEngine, {});
    physics::WorldPtr world = physics::get_world("default");
    ASSERT_NE(nullptr, world);

    // Verify the physics engine
    physics::PhysicsEnginePtr physics = world->Physics();
    ASSERT_NE(nullptr, physics);

    physics::ModelPtr model = world->ModelByName("revolute_model");
    ASSERT_NE(nullptr, model);

    physics::JointPtr joint = model->GetJoint("revolute_joint");
    ASSERT_NE(nullptr, joint);

    // The values in these transforms are based on parameters in the world file
    const ignition::math::Pose3d T1 =
        ignition::math::Pose3d(
          ignition::math::Vector3d(0.0, 0.0, 0.5),
          ignition::math::Quaterniond(0.0, 0.0, 1.570796));

    const ignition::math::Pose3d T2_initial =
        ignition::math::Pose3d(
          ignition::math::Vector3d(0.0, 0.5, 2.0),
          ignition::math::Quaterniond::Identity);

    const ignition::math::Pose3d J_T2_initial =
        ignition::math::Pose3d(
          ignition::math::Vector3d::Zero,
          ignition::math::Quaterniond::Identity);

    // J_T2_initial is the transform from the initial pose of the child link
    // to the pose of the revolute joint.
    const ignition::math::Pose3d J = J_T2_initial + T2_initial;

    const ignition::math::Vector3d axis =
        ignition::math::Vector3d(1.0, 0.0, 0.0);

    joint->SetAxis(0, axis);

    for (const double angleInDegrees : { 0, 30, 90, 117, -64 })
    {
      const double angle = angleInDegrees * IGN_PI / 180.0;
      const ignition::math::Pose3d R =
          ignition::math::Pose3d(
            ignition::math::Vector3d::Zero,
            ignition::math::Quaterniond(axis, angle));

      const ignition::math::Pose3d T2 = J_T2_initial.Inverse() + (R + J);

      joint->SetPosition(0, angle);

      world->Step(1);

      const ignition::math::Pose3d T2_actual = joint->GetChild()->WorldPose();

      VEC_EXPECT_NEAR(T2.Pos(), T2_actual.Pos(), 1e-8);
      VEC_EXPECT_NEAR(T2.Rot().Euler(), T2.Rot().Euler(), 1e-8);
    }
  }

  protected: std::string physicsEngine;
};

/////////////////////////////////////////////////
TEST_P(Issue2505Test, TestAxis)
{
  this->TestAxis();
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue2505Test, PHYSICS_ENGINE_VALUES);


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
