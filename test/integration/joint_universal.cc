/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
  EXPECT_EQ(physics->Type(), _physicsEngine);

  // get model and joints
  physics::ModelPtr model = world->ModelByName("model1");
  ASSERT_TRUE(model != NULL);
  physics::JointPtr jointUpper = model->JointByName("joint00");
  physics::JointPtr jointLower = model->JointByName("joint01");
  ASSERT_TRUE(jointUpper != NULL);
  ASSERT_TRUE(jointLower != NULL);
  physics::LinkPtr linkLower = jointLower->Child();
  ASSERT_TRUE(linkLower != NULL);

  // check joint limits from sdf
  EXPECT_NEAR(1.4, jointLower->HighStop(0).Radian(), g_tolerance);
  EXPECT_NEAR(1.27, jointLower->HighStop(1).Radian(), g_tolerance);
  EXPECT_NEAR(-1.4, jointLower->LowStop(0).Radian(), g_tolerance);
  EXPECT_NEAR(-1.27, jointLower->LowStop(1).Radian(), g_tolerance);

  // freeze upper joint
  jointUpper->SetHighStop(0, ignition::math::Angle(1e-6));
  jointUpper->SetHighStop(1, ignition::math::Angle(1e-6));
  jointUpper->SetLowStop(0, ignition::math::Angle(-1e-6));
  jointUpper->SetLowStop(1, ignition::math::Angle(-1e-6));

  // set asymmetric limits on lower joints
  double hi0 =  0.4;
  double hi1 =  0.2;
  double lo0 = -0.1;
  double lo1 = -0.3;
  jointLower->SetHighStop(0, ignition::math::Angle(hi0));
  jointLower->SetHighStop(1, ignition::math::Angle(hi1));
  jointLower->SetLowStop(0, ignition::math::Angle(lo0));
  jointLower->SetLowStop(1, ignition::math::Angle(lo1));
  EXPECT_NEAR(hi0, jointLower->HighStop(0).Radian(), g_tolerance);
  EXPECT_NEAR(hi1, jointLower->HighStop(1).Radian(), g_tolerance);
  EXPECT_NEAR(lo0, jointLower->LowStop(0).Radian(), g_tolerance);
  EXPECT_NEAR(lo1, jointLower->LowStop(1).Radian(), g_tolerance);

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
          << "pose " << jointLower->Child()->WorldPose()
          << std::endl;
    EXPECT_NEAR(des0, jointLower->Angle(0).Radian(), 1e-2);
    EXPECT_NEAR(des1, jointLower->Angle(1).Radian(), 1e-2);

    // Also test expected pose of body, math is approximate
    ignition::math::Vector3d eulerAngles = linkLower->WorldPose().Rot().Euler();
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
  EXPECT_EQ(physics->Type(), _physicsEngine);

  // get model and joints
  physics::ModelPtr model = world->ModelByName("model1");
  ASSERT_TRUE(model != NULL);
  physics::JointPtr jointLower = model->JointByName("joint01");
  ASSERT_TRUE(jointLower != NULL);

  // Call SetVelocity on both axes of lower joint
  const double vel = 1.0;
  jointLower->SetVelocity(0, vel);
  jointLower->SetVelocity(1, vel);

  // Expect Velocity to match
  EXPECT_NEAR(jointLower->Velocity(0), vel, g_tolerance);
  EXPECT_NEAR(jointLower->Velocity(1), vel, g_tolerance);

  // Expect child link velocity to match parent at joint anchor
  {
    ignition::math::Vector3d childOffset = jointLower->WorldPose().Pos() -
      jointLower->Child()->WorldPose().Pos();
    ignition::math::Vector3d parentOffset = jointLower->WorldPose().Pos() -
      jointLower->Parent()->WorldPose().Pos();
    ignition::math::Quaterniond q;

    ignition::math::Vector3d childVel =
      jointLower->Child()->WorldLinearVel(childOffset, q);
    ignition::math::Vector3d parentVel =
      jointLower->Parent()->WorldLinearVel(parentOffset, q);
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
    gzerr << "DART Universal Joint is not yet working.  See issue #1011.\n";
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
  EXPECT_EQ(physics->Type(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->MaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joint and links
  physics::ModelPtr model1 = world->ModelByName("model1");
  physics::LinkPtr link00 = model1->LinkByName("link00");
  physics::LinkPtr link01 = model1->LinkByName("link01");
  physics::JointPtr joint00 = model1->JointByName("joint00");
  physics::JointPtr joint01 = model1->JointByName("joint01");

  // both initial angles should be zero
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);

  // move child link to it's initial location
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0, 0));
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(0, 1, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0.25*M_PI, 0, 0));
  EXPECT_EQ(joint00->Angle(0), 0.25*M_PI);
  EXPECT_EQ(joint00->Angle(1), 0);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1),
    ignition::math::Vector3d(0, cos(0.25*M_PI), sin(0.25*M_PI)));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "]\n";

  // move child link 45deg about y
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0, 0.25*M_PI, 0));
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0.25*M_PI);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1), ignition::math::Vector3d(0, 1, 0));
  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "]\n";

  // move child link 90deg about both x and "rotated y axis" (z)
  link00->SetWorldPose(ignition::math::Pose3d(0, 0, 2, 0.5*M_PI, 0, 0.5*M_PI));
  EXPECT_EQ(joint00->Angle(1), 0.5*M_PI);
  EXPECT_EQ(joint00->GlobalAxis(0), ignition::math::Vector3d(1, 0, 0));
  EXPECT_EQ(joint00->GlobalAxis(1),
    ignition::math::Vector3d(0, cos(0.5*M_PI), sin(0.5*M_PI)));

  gzdbg << "joint angles [" << joint00->Angle(0)
        << ", " << joint00->Angle(1)
        << "] axis1 [" << joint00->GlobalAxis(0)
        << "] axis2 [" << joint00->GlobalAxis(1)
        << "]\n";

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Universal Joint dynamics broken, see issue #1081.\n";
    return;
  }
  else
  {
    EXPECT_EQ(joint00->Angle(0), 0.5*M_PI);
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
  if (_physicsEngine == "dart")
  {
    gzerr << "DART Universal Joint is not yet working.  See issue #1011.\n";
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
  EXPECT_EQ(physics->Type(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->MaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model1 = world->ModelByName("model1");
  physics::LinkPtr link00 = model1->LinkByName("link00");
  physics::LinkPtr link01 = model1->LinkByName("link01");
  physics::JointPtr joint00 = model1->JointByName("joint00");
  physics::JointPtr joint01 = model1->JointByName("joint01");

  // both initial angles should be zero
  EXPECT_EQ(joint00->Angle(0), 0);
  EXPECT_EQ(joint00->Angle(1), 0);

  // set new upper limit for joint00
  joint00->SetHighStop(0, ignition::math::Angle(0.3));
  // push joint00 till it hits new upper limit
  int count = 0;
  while (joint00->Angle(0) < 0.4 && count < 1220)
  {
    count++;

    joint00->SetForce(0, 0.1);
    world->Step(1);
    // check link pose
    double angle000 = joint00->Angle(0).Radian();
    ignition::math::Pose3d pose00 = link00->WorldPose();
    EXPECT_NEAR(pose00.Rot().Euler().X(), angle000, 1e-8);
    EXPECT_LT(pose00.Rot().Euler().X(), 0.35);
  }

  // push it back to 0 then lock
  joint00->SetLowStop(0, ignition::math::Angle(0.0));
  count = 0;
  while (joint00->Angle(0) > 0.1 && count < 1220)
  {
    count++;

    joint00->SetForce(0, -0.1);
    world->Step(1);
    // check link pose
    double angle000 = joint00->Angle(0).Radian();
    ignition::math::Pose3d pose00 = link00->WorldPose();
    EXPECT_NEAR(pose00.Rot().Euler().X(), angle000, 1e-8);
    EXPECT_GT(pose00.Rot().Euler().X(), -0.05);
  }
  // lock joint at this location by setting lower limit here too
  joint00->SetHighStop(0, ignition::math::Angle(0.0));

  // set joint01 upper limit to 1.0
  joint01->SetHighStop(0, ignition::math::Angle(0.0));
  joint01->SetLowStop(0, ignition::math::Angle(0.0));
  joint01->SetHighStop(1, ignition::math::Angle(2.0));
  // push joint01 until limit is reached
  count = 0;
  while (joint01->Angle(1) < 2.1 && count < 2700)
  {
    count++;

    joint01->SetForce(0, 0.1);
    world->Step(1);

    // check link pose
    ignition::math::Pose3d pose01 = link01->WorldPose();
    double angle001 = joint00->Angle(1).Radian();
    double angle011 = joint01->Angle(1).Radian();

    EXPECT_NEAR(pose01.Rot().Euler().Y(), angle001 + angle011, 1e-8);
    EXPECT_LT(pose01.Rot().Euler().Y(), 2.05);
  }

  // push joint01 the other way until -1 is reached
  joint01->SetLowStop(1, ignition::math::Angle(-1.0));
  count = 0;
  while (joint01->Angle(1) > -1.1 && count < 2100)
  {
    count++;

    joint01->SetForce(1, -0.1);
    world->Step(1);

    // check link pose
    ignition::math::Pose3d pose01 = link01->WorldPose();
    double angle000 = joint00->Angle(0).Radian();
    double angle001 = joint00->Angle(1).Radian();
    double angle010 = joint01->Angle(0).Radian();
    double angle011 = joint01->Angle(1).Radian();

    EXPECT_NEAR(pose01.Rot().Euler().X(), angle000 + angle010, 1e-6);
    EXPECT_NEAR(pose01.Rot().Euler().Y(), angle001 + angle011, 1e-6);
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
