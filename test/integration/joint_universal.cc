/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;

class JointTestUniversal : public ServerFixture,
                           public testing::WithParamInterface<const char*>
{
  /// \brief Test setting and enforcing joint limits.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void Limits(const std::string &_physicsEngine);

  /// \brief Test calling SetVelocity for multiple axes per timestep.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetVelocity(const std::string &_physicsEngine);

  /// \brief Test universal joint implementation with SetWorldPose.
  /// Set links world poses then check joint angles.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void UniversalJointSetWorldPose(const std::string &_physicsEngine);

  /// \brief Test universal joint implementation with forces.
  /// Apply force to universal joint links, check position and/velocity.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void UniversalJointForce(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void JointTestUniversal::Limits(const std::string &_physicsEngine)
{
  // Load our universal joint test world
  Load("worlds/universal_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get model and joints
  physics::ModelPtr model = world->ModelByName("model_1");
  ASSERT_TRUE(model != NULL);
  physics::JointPtr jointUpper = model->GetJoint("joint_00");
  physics::JointPtr jointLower = model->GetJoint("joint_01");
  ASSERT_TRUE(jointUpper != NULL);
  ASSERT_TRUE(jointLower != NULL);
  physics::LinkPtr linkLower = jointLower->GetChild();
  ASSERT_TRUE(linkLower != NULL);

  // check joint limits from sdf
  EXPECT_NEAR(1.4, jointLower->UpperLimit(0), g_tolerance);
  EXPECT_NEAR(1.27, jointLower->UpperLimit(1), g_tolerance);
  EXPECT_NEAR(-1.4, jointLower->LowerLimit(0), g_tolerance);
  EXPECT_NEAR(-1.27, jointLower->LowerLimit(1), g_tolerance);

  // freeze upper joint
  jointUpper->SetUpperLimit(0, 1e-6);
  jointUpper->SetUpperLimit(1, 1e-6);
  jointUpper->SetLowerLimit(0, -1e-6);
  jointUpper->SetLowerLimit(1, -1e-6);
  EXPECT_NEAR(0.0, jointUpper->UpperLimit(0), g_tolerance);
  EXPECT_NEAR(0.0, jointUpper->UpperLimit(1), g_tolerance);
  EXPECT_NEAR(0.0, jointUpper->LowerLimit(0), g_tolerance);
  EXPECT_NEAR(0.0, jointUpper->LowerLimit(1), g_tolerance);
  EXPECT_NEAR(0.0, jointUpper->Position(0), g_tolerance);
  EXPECT_NEAR(0.0, jointUpper->Position(1), g_tolerance);

  // set asymmetric limits on lower joints
  double hi0 =  0.4;
  double hi1 =  0.2;
  double lo0 = -0.1;
  double lo1 = -0.3;
  jointLower->SetUpperLimit(0, hi0);
  jointLower->SetUpperLimit(1, hi1);
  jointLower->SetLowerLimit(0, lo0);
  jointLower->SetLowerLimit(1, lo1);
  EXPECT_NEAR(hi0, jointLower->UpperLimit(0), g_tolerance);
  EXPECT_NEAR(hi1, jointLower->UpperLimit(1), g_tolerance);
  EXPECT_NEAR(lo0, jointLower->LowerLimit(0), g_tolerance);
  EXPECT_NEAR(lo1, jointLower->LowerLimit(1), g_tolerance);

  for (int i = 0; i < 4; ++i)
  {
    // toggle signs for gx, gy
    //     gx gy
    // i=0: +  +
    // i=1: +  -
    // i=2: -  +
    // i=3: -  -
    double gravityMag = 5.0;
    double gx = pow(-1, i / 2) * gravityMag;
    double gy = pow(-1, i % 2) * gravityMag;

    // Set gravity to push horizontally
    physics->SetGravity(ignition::math::Vector3d(gx, gy, 0));
    world->Step(1000);

    // jointLower: axis[0] = {1, 0, 0}
    // jointLower: axis[1] = {0, 1, 0}
    // offset from anchor to c.g. is r = {0, 0, -L}
    // gravity moment r x g

    // a negative gy causes negative rotation about axis[0]
    double des0 = (gy < 0) ? lo0 : hi0;

    // a positive gx causes negative rotation about axis[1]
    double des1 = (gx > 0) ? lo1 : hi1;

    gzdbg << "Setting gravity "
          << "gx " << gx << ' '
          << "gy " << gy << ' '
          << "pose " << jointLower->GetChild()->WorldPose()
          << std::endl;
    EXPECT_NEAR(0.0, jointUpper->Position(0), g_tolerance);
    EXPECT_NEAR(0.0, jointUpper->Position(1), g_tolerance);
    EXPECT_NEAR(des0, jointLower->Position(0), 1e-2);
    EXPECT_NEAR(des1, jointLower->Position(1), 1e-2);

    // Also test expected pose of body, math is approximate
    auto eulerAngles = linkLower->WorldPose().Rot().Euler();
    EXPECT_NEAR(des0, eulerAngles.X(), 0.05);
    EXPECT_NEAR(des1, eulerAngles.Y(), 0.05);
  }
}

/////////////////////////////////////////////////
void JointTestUniversal::SetVelocity(const std::string &_physicsEngine)
{
  // Load our universal joint test world
  Load("worlds/universal_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get model and joints
  physics::ModelPtr model = world->ModelByName("model_1");
  ASSERT_TRUE(model != NULL);
  physics::JointPtr jointLower = model->GetJoint("joint_01");
  ASSERT_TRUE(jointLower != NULL);

  // Call SetVelocity on both axes of lower joint
  const double vel = 1.0;
  jointLower->SetVelocity(0, vel);
  jointLower->SetVelocity(1, vel);

  // Expect GetVelocity to match
  EXPECT_NEAR(jointLower->GetVelocity(0), vel, g_tolerance);
  EXPECT_NEAR(jointLower->GetVelocity(1), vel, g_tolerance);

  // Expect child link velocity to match parent at joint anchor
  {
    auto childOffset = jointLower->WorldPose().Pos() -
      jointLower->GetChild()->WorldPose().Pos();
    auto parentOffset = jointLower->WorldPose().Pos() -
      jointLower->GetParent()->WorldPose().Pos();
    ignition::math::Quaterniond q;

    ignition::math::Vector3d childVel =
      jointLower->GetChild()->WorldLinearVel(childOffset, q);
    ignition::math::Vector3d parentVel =
      jointLower->GetParent()->WorldLinearVel(parentOffset, q);
    EXPECT_NEAR(childVel.X(), parentVel.X(), g_tolerance);
    EXPECT_NEAR(childVel.Y(), parentVel.Y(), g_tolerance);
    EXPECT_NEAR(childVel.Z(), parentVel.Z(), g_tolerance);
  }
}

/////////////////////////////////////////////////
void JointTestUniversal::UniversalJointSetWorldPose(
  const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "DART Universal Joint does not support SetWorldPose.\n";
    return;
  }

  // Load our universal joint test world
  Load("worlds/universal_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d::Zero);

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joint and links
  physics::ModelPtr model_1 = world->ModelByName("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");

  // both initial angles should be zero
  EXPECT_NEAR(joint_00->Position(0), 0, g_tolerance);
  EXPECT_NEAR(joint_00->Position(1), 0, g_tolerance);

  // move child link to it's initial location
  link_00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
  EXPECT_NEAR(joint_00->Position(0), 0, g_tolerance);
  EXPECT_NEAR(joint_00->Position(1), 0, g_tolerance);
  EXPECT_EQ(joint_00->GlobalAxis(0), ignition::math::Vector3d::UnitX);
  EXPECT_EQ(joint_00->GlobalAxis(1), ignition::math::Vector3d::UnitY);
  gzdbg << "joint angles [" << joint_00->Position(0)
        << ", " << joint_00->Position(1)
        << "] axis1 [" << joint_00->GlobalAxis(0)
        << "] axis2 [" << joint_00->GlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  link_00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0.25*IGN_PI, 0, 0));
  EXPECT_NEAR(joint_00->Position(0), 0.25*IGN_PI, g_tolerance);
  EXPECT_NEAR(joint_00->Position(1), 0, g_tolerance);
  EXPECT_EQ(joint_00->GlobalAxis(0), ignition::math::Vector3d::UnitX);
  EXPECT_EQ(joint_00->GlobalAxis(1),
    ignition::math::Vector3d(0, cos(0.25*IGN_PI), sin(0.25*IGN_PI)));
  gzdbg << "joint angles [" << joint_00->Position(0)
        << ", " << joint_00->Position(1)
        << "] axis1 [" << joint_00->GlobalAxis(0)
        << "] axis2 [" << joint_00->GlobalAxis(1)
        << "]\n";

  // move child link 45deg about y
  link_00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0.25*IGN_PI, 0));
  EXPECT_NEAR(joint_00->Position(0), 0, g_tolerance);
  EXPECT_NEAR(joint_00->Position(1), 0.25*IGN_PI, g_tolerance);
  EXPECT_EQ(joint_00->GlobalAxis(0), ignition::math::Vector3d::UnitX);
  EXPECT_EQ(joint_00->GlobalAxis(1), ignition::math::Vector3d::UnitY);
  gzdbg << "joint angles [" << joint_00->Position(0)
        << ", " << joint_00->Position(1)
        << "] axis1 [" << joint_00->GlobalAxis(0)
        << "] axis2 [" << joint_00->GlobalAxis(1)
        << "]\n";

  // move child link 90deg about both x and "rotated y axis" (z)
  link_00->SetWorldPose(ignition::math::Pose3d(
        0, 0, 2, 0.5*IGN_PI, 0, 0.5*IGN_PI));
  EXPECT_NEAR(joint_00->Position(1), 0.5*IGN_PI, g_tolerance);
  EXPECT_EQ(joint_00->GlobalAxis(0), ignition::math::Vector3d::UnitX);
  EXPECT_EQ(joint_00->GlobalAxis(1),
    ignition::math::Vector3d(0, cos(0.5*IGN_PI), sin(0.5*IGN_PI)));

  gzdbg << "joint angles [" << joint_00->Position(0)
        << ", " << joint_00->Position(1)
        << "] axis1 [" << joint_00->GlobalAxis(0)
        << "] axis2 [" << joint_00->GlobalAxis(1)
        << "]\n";

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Universal Joint dynamics broken, see issue #1081.\n";
    return;
  }
  else
  {
    EXPECT_NEAR(joint_00->Position(0), 0.5*IGN_PI, g_tolerance);
  }
}

//////////////////////////////////////////////////
void JointTestUniversal::UniversalJointForce(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Universal Joint dynamics broken, see issue #1081.\n";
    return;
  }

  // Load our universal joint test world
  Load("worlds/universal_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d::Zero);

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // Get model, joints and get links. The model has two links and two universal
  // joints.
  physics::ModelPtr model_1 = world->ModelByName("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");

  // both initial angles should be zero
  EXPECT_NEAR(joint_00->Position(0), 0, g_tolerance);
  EXPECT_NEAR(joint_00->Position(1), 0, g_tolerance);
  EXPECT_NEAR(joint_01->Position(0), 0, g_tolerance);
  EXPECT_NEAR(joint_01->Position(1), 0, g_tolerance);

  // set new upper limit for joint_00
  joint_00->SetUpperLimit(0, 0.3);
  // push joint_00 till it hits new upper limit
  int count = 0;
  while (joint_00->Position(0) < 0.4 && count < 1220)
  {
    count++;

    joint_00->SetForce(0, 0.1);
    world->Step(1);
    // check link pose
    double angle_00_0 = joint_00->Position(0);
    auto pose_00 = link_00->WorldPose();
    EXPECT_NEAR(pose_00.Rot().Euler().X(), angle_00_0, 1e-8);
    EXPECT_LT(pose_00.Rot().Euler().X(), 0.35);
  }

  // push it back to 0 then lock
  joint_00->SetLowerLimit(0, 0.0);
  count = 0;
  while (joint_00->Position(0) > 0.1 && count < 1220)
  {
    count++;

    joint_00->SetForce(0, -0.1);
    world->Step(1);

    // check link pose
    double angle_00_0 = joint_00->Position(0);
    auto pose_00 = link_00->WorldPose();
    EXPECT_NEAR(pose_00.Rot().Euler().X(), angle_00_0, 1e-8);
    EXPECT_GT(pose_00.Rot().Euler().X(), -0.05);
  }

  // Lock joint_00 at this location by setting the upper limit here too
  joint_00->SetUpperLimit(0, 0.0);

  // Lock joint_01-0, but let joint_01-1 go up to 2.0
  joint_01->SetUpperLimit(0, 0.0);
  joint_01->SetLowerLimit(0, 0.0);
  joint_01->SetUpperLimit(1, 2.0);
  // Push joint_01 until limit is reached
  count = 0;
  while (joint_01->Position(1) < 2.1 && count < 2700)
  {
    count++;

    joint_01->SetForce(0, 0.1);
    world->Step(1);

    // check link pose
    auto pose_01 = link_01->WorldPose();
    double angle_00_1 = joint_00->Position(1);
    double angle_01_1 = joint_01->Position(1);

    EXPECT_NEAR(pose_01.Rot().Euler().Y(), angle_00_1 + angle_01_1, 1e-8);
    EXPECT_LT(pose_01.Rot().Euler().Y(), 2.05);
  }

  // The next test assumes that the previous loop drove the positions and
  // velocities to zero, however DART's behavior does not satisfy that
  // assumption (by design), so we set these position and velocity values to
  // zero so that we do not violate the assumptions of the next test.
  for (const physics::JointPtr &j : {joint_00, joint_01})
  {
    for (std::size_t k = 0; k < 2; ++k)
    {
      j->SetVelocity(k, 0.0);
      j->SetPosition(k, 0.0);
    }
  }

  // push joint_01 the other way until -1 is reached
  // test whether the combined angles of the two joints match the Euler Angles
  // of the expected pose
  joint_01->SetLowerLimit(1, -1.0);
  count = 0;
  while (joint_01->Position(1) > -1.1 && count < 2100)
  {
    count++;

    joint_01->SetForce(1, -0.1);
    world->Step(1);

    // check link pose
    auto pose_01 = link_01->WorldPose();
    double angle_00_0 = joint_00->Position(0);
    double angle_00_1 = joint_00->Position(1);
    double angle_01_0 = joint_01->Position(0);
    double angle_01_1 = joint_01->Position(1);


    // Note that these tests only succeed if the initial positions and
    // velocities of the first axis of each joint start out at zero. If the
    // first axis of either joint exhibits any non-zero position, then these
    // tests will fail.
    EXPECT_NEAR(0.0, angle_00_0, 1e-6);
    EXPECT_NEAR(0.0, angle_01_0, 1e-6);
    EXPECT_NEAR(0.0, pose_01.Rot().Euler().X(), 1e-6);
    EXPECT_NEAR(pose_01.Rot().Euler().Y(), angle_00_1 + angle_01_1, 1e-6);
  }
}

/////////////////////////////////////////////////
TEST_P(JointTestUniversal, Limits)
{
  Limits(GetParam());
}

/////////////////////////////////////////////////
TEST_P(JointTestUniversal, SetVelocity)
{
  SetVelocity(GetParam());
}

/////////////////////////////////////////////////
TEST_P(JointTestUniversal, UniversalJointSetWorldPose)
{
  UniversalJointSetWorldPose(GetParam());
}

/////////////////////////////////////////////////
TEST_P(JointTestUniversal, UniversalJointForce)
{
  UniversalJointForce(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestUniversal,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
