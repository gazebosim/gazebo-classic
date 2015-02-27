/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/physics.hh"
// #include "gazebo/physics/Joint.hh"
// #include "gazebo/physics/ScrewJoint.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;
const double g_tolerance = 1e-2;

class JointTestScrew : public JointTest
{
  /// \brief Test screw joint implementation with SetWorldPose.
  /// Set link poses in world frame, check joint angles and joint axis.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointSetWorldPose(const std::string &_physicsEngine);

  /// \brief Test screw joint implementation with forces.
  /// Apply force to screw joint links, check velocity.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointForce(const std::string &_physicsEngine);

  /// \brief Test screw joint limits implementation with forces.
  /// Apply force to screw joint links, check velocity.
  /// Keep increasing force until something gives
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void ScrewJointLimitForce(const std::string &_physicsEngine);

  /// \brief Spin joints several rotations and verify that the angles
  /// wrap properly.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void WrapAngle(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////
void JointTestScrew::WrapAngle(const std::string &_physicsEngine)
{
  /// \TODO: bullet hinge angles are wrapped (#1074)
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test for bullet, see issues #1074.\n";
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

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  {
    std::string jointType = "screw";
    gzdbg << "SpawnJoint " << jointType << " child world" << std::endl;
    physics::JointPtr joint = SpawnJoint(jointType, false, true);
    ASSERT_TRUE(joint != NULL);

    // \TODO: option to set thread pitch, create another test
    // double threadPitch = 100.0;
    // joint->SetParam("thread_pitch", threadPitch);

    // Inertial parameters
    const double momentOfInertia = 1.0;
    const double threadPitch = 1.0;
    const double mass = 1.0;
    double inertia = momentOfInertia + mass / (threadPitch*threadPitch);

    // Verify inertial parameters
    EXPECT_NEAR(threadPitch, joint->GetParam("thread_pitch", 0), g_tolerance);
    {
      physics::LinkPtr child = joint->GetChild();
      EXPECT_NEAR(mass, child->GetInertial()->GetMass(), g_tolerance);
    }
    /// \TODO: verify momentOfInertia

    // set torque and step forward
    const double torque = 35;
    const unsigned int stepCount = 1000;
    double dt = physics->GetMaxStepSize();
    double stepTime = stepCount * dt;

    // Expect constant torque to give quadratic response in position
    {
      // Expected max joint angle (quatratic in time)
      math::Angle maxAngle(0.5 * torque * stepTime*stepTime / inertia);
      // Verify that the joint should make more than 1 revolution
      EXPECT_GT(maxAngle.Radian(), 1.25 * 2 * M_PI);
    }

    // compute joint velocity analytically with constant torque
    // joint angle is unwrapped
    for (unsigned int i = 0; i < stepCount; ++i)
    {
      joint->SetForce(0, torque);

      double vel = sqrt(2.0*torque*joint->GetAngle(0).Radian() / inertia);
      world->Step(1);
      EXPECT_NEAR(joint->GetVelocity(0), vel, 2e-2);
      double time = world->GetSimTime().Double();
      math::Angle angle(0.5 * torque * time*time / inertia);
      EXPECT_NEAR(joint->GetAngle(0).Radian(), angle.Radian(), g_tolerance);
    }
    std::cout << "Final time:  " << world->GetSimTime().Double() << std::endl;
    std::cout << "Final angle: " << joint->GetAngle(0).Radian() << std::endl;
    std::cout << "Final speed: " << joint->GetVelocity(0) << std::endl;
  }
}

TEST_P(JointTestScrew, WrapAngle)
{
  WrapAngle(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointSetWorldPose(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "DART Screw Joint will not work with Link::SetWorldPose."
          << " See issue #1096.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Screw Joint will not work with Link::SetWorldPose."
          << " See issue #857.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/screw_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");

  // both initial angles should be zero
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);

  // move child link to it's initial location
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0, 0));
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  double pitch_00 = joint_00->GetParam("thread_pitch", 0);
  math::Pose pose_00 = math::Pose(-0.25*M_PI/pitch_00, 0, 2, 0.25*M_PI, 0, 0);
  math::Pose pose_01 = math::Pose(0, 0, -1, 0, 0, 0) + pose_00;
  link_00->SetWorldPose(pose_00);
  link_01->SetWorldPose(pose_01);
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), -0.25*M_PI/pitch_00);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "]\n";

  // move child link 45deg about y
  double pitch_01 = joint_01->GetParam("thread_pitch", 0);
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0.25*M_PI, 0));
  pose_00 = math::Pose(-0.25*M_PI/pitch_00, 0, 2, 0.25*M_PI, 0, 0);
  pose_01 = math::Pose(-0.3*M_PI/pitch_01, 0, -1, 0.3*M_PI, 0, 0) + pose_00;
  link_00->SetWorldPose(pose_00);
  link_01->SetWorldPose(pose_01);
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), -0.25*M_PI/pitch_00);
  EXPECT_EQ(joint_01->GetAngle(0), 0.3*M_PI);
  EXPECT_EQ(joint_01->GetAngle(1), -0.3*M_PI/pitch_01);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "] pitch_01 [" << pitch_01
        << "]\n";

  // new poses should not violate the constraint.  take a few steps
  // and make sure nothing moves.
  world->Step(10);

  // move child link 90deg about both x and "rotated y axis" (z)
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), -0.25*M_PI/pitch_00);
  EXPECT_EQ(joint_01->GetAngle(0), 0.3*M_PI);
  EXPECT_EQ(joint_01->GetAngle(1), -0.3*M_PI/pitch_01);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(1, 0, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "] pitch_00 [" << pitch_00
        << "] pitch_01 [" << pitch_01
        << "]\n";
}

TEST_P(JointTestScrew, ScrewJointSetWorldPose)
{
  ScrewJointSetWorldPose(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointForce(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    /// \TODO skipping bullet, see issue #1081
    gzerr << "BulletScrewJoint::GetAngle() is one step behind (issue #1081).\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for dart, see issues #1096.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/screw_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joints and get links
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");
  double pitch_00 = joint_00->GetParam("thread_pitch", 0);
  double pitch_01 = joint_01->GetParam("thread_pitch", 0);

  // both initial angles should be zero
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);

  // set new upper limit for joint_00
  joint_00->SetHighStop(0, 0.3);
  bool once = false;
  int count = 0;
  int maxCount = 5000;
  // push joint_00 till it hits new upper limit
  while (count < maxCount && joint_00->GetAngle(0) < 0.3)
  {
    joint_00->SetForce(0, 0.1);
    world->Step(1);
    ++count;
    // check link pose
    double angle_00_angular = joint_00->GetAngle(0).Radian();
    double angle_00_linear = joint_00->GetAngle(1).Radian();
    double angle_01_linear = joint_01->GetAngle(1).Radian();
    math::Pose pose_01 = link_01->GetWorldPose();
    EXPECT_EQ(link_00->GetWorldPose(),
      math::Pose(-angle_00_angular / pitch_00, 0, 2, angle_00_angular, 0, 0));

    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint_00 " << angle_00_linear
              << " shoudl be 0.3\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose_01.pos.x, angle_00_linear + angle_01_linear, 1e-8);
    }
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  double maxOvershootRadians = 0.01;
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint_00->SetForce(0, 0.1);
    world->Step(1);
    EXPECT_LT(joint_00->GetAngle(0), 0.3 + maxOvershootRadians);
  }


  // lock joint at this location by setting lower limit here too
  joint_00->SetLowStop(0, 0.3);

  // set joint_01 upper limit to 1.0
  joint_01->SetHighStop(0, 1.0);

  // push joint_01 until limit is reached
  once = false;
  count = 0;
  while (count < maxCount && joint_01->GetAngle(0) < 1.0)
  {
    joint_01->SetForce(0, 0.1);
    world->Step(1);
    ++count;

    // check link pose
    math::Pose pose_00 = link_00->GetWorldPose();
    math::Pose pose_01 = link_01->GetWorldPose();
    double angle_00_angular = joint_00->GetAngle(0).Radian();
    double angle_00_linear = joint_00->GetAngle(1).Radian();
    double angle_01_angular = joint_01->GetAngle(0).Radian();
    double angle_01_linear = joint_01->GetAngle(1).Radian();

    EXPECT_EQ(pose_00, math::Pose(
      -angle_00_angular / pitch_00, 0, 2, angle_00_angular, 0, 0));
    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint_00 " << angle_00_linear
              << " should be 0.3. "
              << " joint_01 " << angle_01_linear
              << " is off too.\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose_01.pos.x, angle_00_linear + angle_01_linear, 1e-8);
    }
    EXPECT_NEAR(pose_01.pos.x,
      -angle_00_angular / pitch_00 - angle_01_angular / pitch_01, 1e-8);
    EXPECT_NEAR(pose_01.rot.GetAsEuler().x,
      angle_00_angular + angle_01_angular, 1e-8);
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint_01->SetForce(0, 0.1);
    world->Step(1);
    EXPECT_LT(joint_01->GetAngle(0), 1.0 + maxOvershootRadians);
  }

  // push joint_01 the other way until -1 is reached
  once = false;
  count = 0;
  while (count < maxCount && joint_01->GetAngle(0) > -1.0)
  {
    joint_01->SetForce(0, -0.1);
    world->Step(1);
    ++count;

    // check link pose
    math::Pose pose_00 = link_00->GetWorldPose();
    math::Pose pose_01 = link_01->GetWorldPose();
    double angle_00_angular = joint_00->GetAngle(0).Radian();
    double angle_00_linear = joint_00->GetAngle(1).Radian();
    double angle_01_angular = joint_01->GetAngle(0).Radian();
    double angle_01_linear = joint_01->GetAngle(1).Radian();

    EXPECT_EQ(pose_00, math::Pose(
      -angle_00_angular / pitch_00, 0, 2, angle_00_angular, 0, 0));
    if (_physicsEngine == "simbody")
    {
      if (!once)
      {
        once = true;
        gzerr << "skip test: issue #857 in simbody screw joint linear angle:"
              << " joint_00 " << angle_00_linear
              << " should be 0.3. "
              << " joint_01 " << angle_01_linear
              << " is off too.\n";
      }
    }
    else
    {
      EXPECT_NEAR(pose_01.pos.x, angle_00_linear + angle_01_linear, 1e-8);
    }
    EXPECT_NEAR(pose_01.pos.x,
      -angle_00_angular / pitch_00 - angle_01_angular / pitch_01, 1e-8);
    EXPECT_NEAR(pose_01.rot.GetAsEuler().x,
      angle_00_angular + angle_01_angular, 1e-8);
  }
  gzdbg << "took [" << count << "] steps.\n";

  // continue pushing for 1000 steps to make sure there is no overshoot
  joint_01->SetLowStop(0, -1.0);
  for (unsigned int i = 0; i < 1000; ++i)
  {
    joint_01->SetForce(0, -0.1);
    world->Step(1);
    EXPECT_GT(joint_01->GetAngle(0), -1.0 - maxOvershootRadians);
  }
}

TEST_P(JointTestScrew, ScrewJointForce)
{
  ScrewJointForce(this->physicsEngine);
}

//////////////////////////////////////////////////
void JointTestScrew::ScrewJointLimitForce(const std::string &_physicsEngine)
{
  // Load pr2 world
  ServerFixture::Load("worlds/pr2.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // get model, joints and get links
  physics::ModelPtr model = world->GetModel("pr2");
  physics::LinkPtr link_00 = model->GetLink("torso_lift_link");

  // drop from some height
  model->SetWorldPose(math::Pose(0, 0, 0.5, 0, 0, 0));
  // +1sec: should have hit the ground
  world->Step(1000);
  // +4sec: should destabilize without patch for issue #1159
  world->Step(4000);

  for (int n = 0; n < 1000; ++n)
  {
    world->Step(1);
    math::Vector3 vel_angular = link_00->GetWorldLinearVel();
    math::Vector3 vel_linear = link_00->GetWorldAngularVel();

    EXPECT_LT(vel_angular.GetLength(), 0.1);
    EXPECT_LT(vel_linear.GetLength(), 0.1);

    gzlog <<   "va [" << vel_angular
          << "] vl [" << vel_linear
          << "]\n";
  }
}

TEST_P(JointTestScrew, ScrewJointLimitForce)
{
  ScrewJointLimitForce(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestScrew,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("screw")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
