/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <chrono>
#include <thread>
#include <random>

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/util.hh"

#define TOL 0.001
using namespace gazebo;

class JointKinematicTest : public ServerFixture,
                           public ::testing::WithParamInterface<const char*>
{
  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in series with World::Step(1) with physics paused to avoid race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionTest(const std::string &_physicsEngine);

  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in parallel with World::Step(1) with physics running to check for race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionThreadedTest(const std::string &_physicsEngine);

  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in series with World::Step(1) with physics paused to avoid race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// This test tries to set joint angles of loop joints, the model state
  /// should not change.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionLoopJointTest(const std::string &_physicsEngine);

  /// \brief Test setting joint position while the model is translating.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetPositionTranslating(const std::string &_physicsEngine);

  /// \brief Test setting joint position while the child link is rotating.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetPositionRotating(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionTest(const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  std::default_random_engine seed;

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double test_wall_duration = 2.0;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_1::joint_01"));
  joints.push_back(model->GetJoint("model_1::joint_12"));
  // joints.push_back(model->GetJoint("model_1::joint_23"));
  // joints.push_back(model->GetJoint("model_1::joint_34"));
  // joints.push_back(model->GetJoint("model_1::joint_45"));
  // joints.push_back(model->GetJoint("model_1::joint_52"));
  joints.push_back(model->GetJoint("model_1::joint_22a"));
  joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  // joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  // joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  // joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  // joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  world->SetPaused(true);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << test_wall_duration
        << " secs, see how well Joint::SetPosition deals with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition in series"
        << " with World::Step, so there's no physics engine update collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  while (common::Time::GetWallTime().Double() <
         start_wall_time + test_wall_duration)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        std::uniform_real_distribution<double> rnd_dist(0.0, 1.0);
        (*ji)->SetPosition(0, rnd_dist(seed));
      }

      // gzdbg << "debug: running ["
      //       << common::Time::GetWallTime().Double()
      //       << " / " << start_wall_time + test_wall_duration
      //       << "]\n";
    }

    // step simulation
    world->Step(1);

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li)
    {
      ignition::math::Vector3d linVel = (*li)->WorldLinearVel();
      ignition::math::Vector3d angVel = (*li)->WorldAngularVel();
      EXPECT_NEAR(linVel.X(), 0, TOL);
      EXPECT_NEAR(linVel.Y(), 0, TOL);
      EXPECT_NEAR(linVel.Z(), 0, TOL);
      EXPECT_NEAR(angVel.X(), 0, TOL);
      EXPECT_NEAR(angVel.Y(), 0, TOL);
      EXPECT_NEAR(angVel.Z(), 0, TOL);
    }
  }
  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

TEST_P(JointKinematicTest, SetJointPositionTest)
{
  SetJointPositionTest(GetParam());
}

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionThreadedTest(
  const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  std::default_random_engine seed;

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double test_wall_duration = 2.0;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_1::joint_01"));
  joints.push_back(model->GetJoint("model_1::joint_12"));
  // joints.push_back(model->GetJoint("model_1::joint_23"));
  // joints.push_back(model->GetJoint("model_1::joint_34"));
  // joints.push_back(model->GetJoint("model_1::joint_45"));
  // joints.push_back(model->GetJoint("model_1::joint_52"));
  joints.push_back(model->GetJoint("model_1::joint_22a"));
  joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  // joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  // joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  // joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  // joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  world->SetPaused(false);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << test_wall_duration
        << " secs, see how well Joint::SetPosition delas with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition happens"
        << " in parallel with phsics update, leading to potential collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  while (common::Time::GetWallTime().Double() <
         start_wall_time + test_wall_duration)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        std::uniform_real_distribution<double> rnd_dist(0.0, 1.0);
        (*ji)->SetPosition(0, rnd_dist(seed));
      }

      // gzdbg << "debug: running ["
      //       << common::Time::GetWallTime().Double()
      //       << " / " << start_wall_time + test_wall_duration
      //       << "]\n";
    }

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li)
    {
      ignition::math::Vector3d linVel = (*li)->WorldLinearVel();
      ignition::math::Vector3d angVel = (*li)->WorldAngularVel();
      EXPECT_NEAR(linVel.X(), 0, TOL);
      EXPECT_NEAR(linVel.Y(), 0, TOL);
      EXPECT_NEAR(linVel.Z(), 0, TOL);
      EXPECT_NEAR(angVel.X(), 0, TOL);
      EXPECT_NEAR(angVel.Y(), 0, TOL);
      EXPECT_NEAR(angVel.Z(), 0, TOL);
    }
  }
  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

// This test fails on OSX (see issue #1219)
// https://bitbucket.org/osrf/gazebo/issues/1219
#ifndef __APPLE__
TEST_P(JointKinematicTest, SetJointPositionThreadedTest)
{
  SetJointPositionThreadedTest(GetParam());
}
#endif

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionLoopJointTest(
    const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  std::default_random_engine seed;

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double iterations = 3;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  // joints.push_back(model->GetJoint("model_1::joint_01"));
  // joints.push_back(model->GetJoint("model_1::joint_12"));
  joints.push_back(model->GetJoint("model_1::joint_23"));
  joints.push_back(model->GetJoint("model_1::joint_34"));
  joints.push_back(model->GetJoint("model_1::joint_45"));
  joints.push_back(model->GetJoint("model_1::joint_52"));
  // joints.push_back(model->GetJoint("model_1::joint_22a"));
  // joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  std::vector<ignition::math::Pose3d> linkPoses;
  for (physics::Link_V::iterator li = links.begin();
                                 li != links.end(); ++li)
    linkPoses.push_back((*li)->WorldPose());

  world->SetPaused(true);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << iterations
        << " steps, see how well Joint::SetPosition delas with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition in series"
        << " with World::Step, so there's no physics engine update collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  for (int i = 0; i < iterations; ++i)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        std::uniform_real_distribution<double> rnd_dist(0.0, 1.0);
        (*ji)->SetPosition(0, rnd_dist(seed));
      }

      // gzdbg << "debug: running ["
      //       << i << " / " << iterations
      //       << "]\n";
    }

    // step simulation
    world->Step(1);

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    auto pi = linkPoses.begin();
    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li, ++pi)
    {
      auto pose = (*li)->WorldPose();
      EXPECT_NEAR(pose.Pos().X(), pi->Pos().X(), TOL);
      EXPECT_NEAR(pose.Pos().Y(), pi->Pos().Y(), TOL);
      EXPECT_NEAR(pose.Pos().Z(), pi->Pos().Z(), TOL);
      EXPECT_NEAR(pose.Rot().W(), pi->Rot().W(), TOL);
      EXPECT_NEAR(pose.Rot().X(), pi->Rot().X(), TOL);
      EXPECT_NEAR(pose.Rot().Y(), pi->Rot().Y(), TOL);
      EXPECT_NEAR(pose.Rot().Z(), pi->Rot().Z(), TOL);
    }
  }

  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

TEST_P(JointKinematicTest, SetJointPositionLoopJointTest)
{
  SetJointPositionLoopJointTest(GetParam());
}

//////////////////////////////////////////////////
void JointKinematicTest::SetPositionTranslating(
    const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "BulletLink::AddForce is not implemented (issue #1476).\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Two boxes connected by a revolute joint at the center of the second one
  Load("test/worlds/set_joint_position_moving.world", true, _physicsEngine);

  // Verify world
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  auto physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  const double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // Verify model
  auto model = world->ModelByName("model");
  ASSERT_TRUE(model != nullptr);

  auto link0 = model->GetLink("link0");
  auto link1 = model->GetLink("link1");
  ASSERT_TRUE(link0 != nullptr);
  ASSERT_TRUE(link1 != nullptr);

  auto joint = model->GetJoint("joint");
  ASSERT_TRUE(joint != nullptr);

  const int accSteps = 800;
  const auto force = ignition::math::Vector3d::UnitX;
  const auto zeroVec = ignition::math::Vector3d::Zero;

  // Calculate expected linear velocity after steps
  const auto linVel = (accSteps * dt) * 2 * force /
    (link0->GetInertial()->Mass() + link1->GetInertial()->Mass());

  for (int i = 0; i < accSteps; ++i)
  {
    link0->AddForce(force);
    link1->AddForce(force);
    world->Step(1);
  }

  // Both links have reached expected velocity
  VEC_EXPECT_NEAR(linVel, link0->WorldLinearVel(), TOL);
  VEC_EXPECT_NEAR(linVel, link1->WorldLinearVel(), TOL);
  VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
  VEC_EXPECT_NEAR(zeroVec, link1->RelativeAngularVel(), TOL);

  // Set the joint angle
  EXPECT_NEAR(0.0, joint->Position(0), TOL);
  double newAngle = 0.9;
  // Preserve the world velocity when SetPosition
  joint->SetPosition(0, newAngle, true);
  world->Step(1);

  // Check that child link pose changed
  EXPECT_NEAR(newAngle, joint->Position(0), TOL);
  EXPECT_NEAR(0.0, link0->WorldPose().Rot().Euler().Z(), TOL);
  EXPECT_NEAR(newAngle, link1->WorldPose().Rot().Euler().Z(), TOL);

  // Expect velocities to be unchanged
  VEC_EXPECT_NEAR(linVel, link0->WorldLinearVel(), TOL);
  VEC_EXPECT_NEAR(linVel, link1->WorldLinearVel(), TOL);
  VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
  VEC_EXPECT_NEAR(zeroVec, link1->RelativeAngularVel(), TOL);

  // Save the parent link velocity for later testing
  const auto previousParentLinkVel = link0->WorldLinearVel();

  newAngle = 0.6;
  // Do not preserve the world velocity (default behavior)
  joint->SetPosition(0, newAngle);

  if (_physicsEngine == "ode" || _physicsEngine == "bullet")
  {
    // The expected behavior here is undefined for DART and Simbody, so we do
    // not test it on those simulators.

    // Check that child link pose changed
    EXPECT_NEAR(newAngle, joint->Position(0), TOL);
    EXPECT_NEAR(0.0, link0->WorldPose().Rot().Euler().Z(), TOL);
    EXPECT_NEAR(newAngle, link1->WorldPose().Rot().Euler().Z(), TOL);

    // Expect child velocities to be zero
    VEC_EXPECT_NEAR(zeroVec, link1->WorldLinearVel(), TOL);
    VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
    VEC_EXPECT_NEAR(zeroVec, link1->RelativeAngularVel(), TOL);
  }

  // Expect the parent velocity to be unchanged
  VEC_EXPECT_NEAR(previousParentLinkVel, link0->WorldLinearVel(), TOL);
}

TEST_P(JointKinematicTest, SetPositionTranslating)
{
  SetPositionTranslating(GetParam());
}

//////////////////////////////////////////////////
void JointKinematicTest::SetPositionRotating(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "BulletLink::AddTorque is not implemented (issue #1476).\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Two boxes connected by a revolute joint at the center of the second one
  Load("test/worlds/set_joint_position_moving.world", true, _physicsEngine);

  // Verify world
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  auto physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  const double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // Verify model
  auto model = world->ModelByName("model");
  ASSERT_TRUE(model != nullptr);

  auto link0 = model->GetLink("link0");
  auto link1 = model->GetLink("link1");
  ASSERT_TRUE(link0 != nullptr);
  ASSERT_TRUE(link1 != nullptr);

  auto joint = model->GetJoint("joint");
  ASSERT_TRUE(joint != nullptr);

  const int accSteps = 800;
  const auto torque = ignition::math::Vector3d::UnitZ;
  const auto zeroVec = ignition::math::Vector3d::Zero;

  // Calculate expected angular velocity after steps
  const auto angVel = (accSteps * dt) * torque /
    link1->GetInertial()->PrincipalMoments();

  for (int i = 0; i < accSteps; ++i)
  {
    link1->AddTorque(torque);
    world->Step(1);
  }

  // Expect second link to rotate and first to not rotate
  VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
  VEC_EXPECT_NEAR(angVel, link1->RelativeAngularVel(), TOL);
  EXPECT_NEAR(angVel.Z(), joint->GetVelocity(0), TOL);

  // Set the joint position back to zero
  EXPECT_LT(0.0, joint->Position(0));
  // Preserve the world velocity when SetPosition
  joint->SetPosition(0, 0.0, true);
  world->Step(1);

  // Expect velocities to be unchanged
  VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
  VEC_EXPECT_NEAR(angVel, link1->RelativeAngularVel(), TOL);
  EXPECT_NEAR(angVel.Z(), joint->GetVelocity(0), TOL);

  // Reset physics states before setPosition again.
  // Do not preserve the world velocity (default behavior)
  joint->SetPosition(0, 1.0);
  world->Step(1);

  if (_physicsEngine == "ode" || _physicsEngine == "bullet")
  {
    // ODE and Bullet should reset the velocities after calling
    // Joint::SetPosition with _preserveWorldVelocity set to false.
    // For other physics engines, the behavior is undefined.
    VEC_EXPECT_NEAR(zeroVec, link0->WorldLinearVel(), TOL);
    VEC_EXPECT_NEAR(zeroVec, link1->WorldLinearVel(), TOL);
    VEC_EXPECT_NEAR(zeroVec, link0->RelativeAngularVel(), TOL);
    VEC_EXPECT_NEAR(zeroVec, link1->RelativeAngularVel(), TOL);
    EXPECT_NEAR(0.0, joint->GetVelocity(0), TOL);
  }
}

TEST_P(JointKinematicTest, SetPositionRotating)
{
  SetPositionRotating(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointKinematicTest,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
