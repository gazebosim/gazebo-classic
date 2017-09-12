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
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3Stats.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include "test/util.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;

class PhysicsLinkTest : public ServerFixture,
                        public ::testing::WithParamInterface<const char*>
{
  /// \brief Test force adding functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void AddForce(const std::string &_physicsEngine);

  /// \brief Use AddLinkForce on the given direction and then the opposite
  /// direction so they cancel out.
  /// \param[in] _physicsEngine Name of the physics engine that is being used
  /// \param[in] _world_equals_link World Frame == Link Frame
  /// \param[in] _link_equals_inertial Link Frame == Inertial Frame
  /// \param[in] _world World pointer.
  /// \param[in] _link Link pointer.
  /// \param[in] _force Force expressed in link frame.
  /// \param[in] _offset Offset expressed in link frame, defaults to link
  /// origin.
  public: void AddLinkForceTwoWays(
      const std::string &_physicsEngine,
      const bool _world_equals_link,
      const bool _link_equals_inertial,
      physics::WorldPtr _world,
      physics::LinkPtr _link, ignition::math::Vector3d _force,
      ignition::math::Vector3d _offset = ignition::math::Vector3d::Zero);

  /// \brief Test GetWorldAngularMomentum.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetWorldAngularMomentum(const std::string &_physicsEngine);

  /// \brief Test GetWorldEnergy* functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetWorldEnergy(const std::string &_physicsEngine);

  /// \brief Test Link::GetWorldInertia* functions.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void GetWorldInertia(const std::string &_physicsEngine);

  /// \brief Test wrench subscriber.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void OnWrenchMsg(const std::string &_physicsEngine);

  /// \brief Test velocity setting functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetVelocity(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsLinkTest::AddLinkForceTwoWays(
    const std::string& _physicsEngine,
    const bool _world_equals_link,
    const bool _link_equals_inertial,
    physics::WorldPtr _world,
    physics::LinkPtr _link, ignition::math::Vector3d _force,
    ignition::math::Vector3d _offset)
{
  // Get state before adding force
  ignition::math::Vector3d linearVelWorld0 = _link->WorldCoGLinearVel();
  ignition::math::Vector3d angularVelWorld0 = _link->WorldAngularVel();
  ignition::math::Pose3d poseWorld0 = _link->WorldPose();

  // Add Link Force
  if (_offset == ignition::math::Vector3d::Zero)
    _link->AddLinkForce(_force);
  else
    _link->AddLinkForce(_force, _offset);

  const int moreThanOneStep = 2;
  const double dt = _world->Physics()->GetMaxStepSize();

  // Note: This step must be performed before checking the link forces when ODE
  // is the physics engine, because otherwise the link accelerations will not be
  // computed, and therefore the result of WorldForce and WorldTorque (which
  // depend on the current accelerations) will be zero.
  //
  // However, other simulation engines (such as DART) maintain the link forces
  // and torques as part of the current state, and the values (might) get
  // cleared out after each simulation step (depending on the engine's current
  // settings), so it should not necessarily be called for each engine. If this
  // test is enabled for another engine (such as SimBody or Bullet), then that
  // engine should perform its simulation step either in this if-statement or
  // the one below depending on how its simulation procedure works.
  if ("ode" == _physicsEngine)
  {
    _world->Step(1);
  }

  // Check force and torque relative to the COG in world coordinates
  ignition::math::Vector3d forceWorld = poseWorld0.Rot().RotateVector(_force);
  EXPECT_EQ(forceWorld, _link->WorldForce());

  ignition::math::Vector3d worldOffset = poseWorld0.Rot().RotateVector(
      _offset - _link->GetInertial()->CoG());
  ignition::math::Vector3d torqueWorld = worldOffset.Cross(forceWorld);
  EXPECT_EQ(torqueWorld, _link->WorldTorque());

  // Check acceleration in world frame
  ignition::math::Vector3d oneStepLinearAccel =
      forceWorld/_link->GetInertial()->Mass();
  EXPECT_EQ(oneStepLinearAccel, _link->WorldLinearAccel());

  // Compute angular accel by multiplying world torque
  // by inverse of world inertia matrix.
  // In this case, the gyroscopic coupling terms are zero
  // since the model is a unit box.
  ignition::math::Vector3d oneStepAngularAccel =
      _link->WorldInertiaMatrix().Inverse() * torqueWorld;
  EXPECT_EQ(oneStepAngularAccel, _link->WorldAngularAccel());


  // Note: This step must be performed after checking the link forces when DART
  // is the physics engine, because otherwise the accelerations used by the
  // previous tests will be cleared out before they can be tested.
  if ("dart" == _physicsEngine)
  {
    _world->Step(1);
  }

  // Check velocity in world frame
  ignition::math::Vector3d oneStepLinearVel = linearVelWorld0 +
    dt*oneStepLinearAccel;

  // Dev note (MXG): DART does not always produce quite the same result as the
  // expected value for CoG linear velocity. It might be worth investigating
  // whether this is ordinary numerical error or if DART should be tweaked to be
  // more precise.
  //
  // The tests succeed for a tolerance as low as 1e-6 in all trials except
  // "World != link != inertial frame". This makes me suspect that the
  // inaccuracy is a result of cummulative matrix multiplications.
  double tolerance = 1e-6;
  if ("dart" == _physicsEngine && !_world_equals_link && !_link_equals_inertial)
  {
    tolerance = 2e-3;
  }

  VEC_EXPECT_NEAR(oneStepLinearVel, _link->WorldCoGLinearVel(), tolerance);

  ignition::math::Vector3d oneStepAngularVel = angularVelWorld0 +
    dt*oneStepAngularAccel;
  EXPECT_EQ(oneStepAngularVel, _link->WorldAngularVel());

  // Step forward and check again
  _world->Step(moreThanOneStep);

  // Check that force and torque are zero
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldForce());
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldTorque());

  // Check that acceleration is zero
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldLinearAccel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldAngularAccel());

  // Check that velocity hasn't changed
  // Dev note (MXG): Same note as above regarding the tolerance that is used
  // here. Everything but "World != link != inertial frame" succeeds with 1e-6.
  VEC_EXPECT_NEAR(oneStepLinearVel, _link->WorldCoGLinearVel(), tolerance);
  EXPECT_EQ(oneStepAngularVel, _link->WorldAngularVel());

  // Add opposing force in link frame and check that link is back to initial
  // velocity
  if (_offset == ignition::math::Vector3d::Zero)
    _link->AddLinkForce(-_force);
  else
    _link->AddLinkForce(-_force, _offset);

  _world->Step(moreThanOneStep);
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldForce());
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldTorque());
  EXPECT_EQ(linearVelWorld0, _link->WorldCoGLinearVel());
  EXPECT_EQ(angularVelWorld0, _link->WorldAngularVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldLinearAccel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, _link->WorldAngularAccel());
}

/////////////////////////////////////////////////
void PhysicsLinkTest::AddForce(const std::string &_physicsEngine)
{
  // TODO bullet and simbody currently fail this test
  if (_physicsEngine != "ode" && _physicsEngine != "dart")
  {
    gzerr << "Aborting AddForce test for Bullet and Simbody. "
          << "See issues #1476 and #1478."
          << std::endl;
    return;
  }

  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // disable gravity
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Spawn a box
  ignition::math::Vector3d size(1, 1, 1);
  SpawnBox("box", size, ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero, false);
  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Check that link is at rest
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearAccel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularAccel());

  // Add force at link frame
  gzdbg << "World == link == inertial frames, no offset" << std::endl;
  EXPECT_EQ(ignition::math::Pose3d::Zero, link->WorldPose());
  EXPECT_EQ(ignition::math::Pose3d::Zero, link->WorldInertialPose());
  this->AddLinkForceTwoWays(_physicsEngine, true, true, world, link,
                            ignition::math::Vector3d(1, 20, 31));

  gzdbg << "World != link == inertial frames, no offset" << std::endl;
  model->SetLinkWorldPose(ignition::math::Pose3d(
        ignition::math::Vector3d(2, 3, 4),
        ignition::math::Quaterniond(0, IGN_PI/2.0, 1)), link);
  EXPECT_NE(ignition::math::Pose3d::Zero, link->WorldPose());
  EXPECT_EQ(link->WorldPose(), link->WorldInertialPose());
  this->AddLinkForceTwoWays(_physicsEngine, false, true, world, link,
                            ignition::math::Vector3d(-1, 10, 5));

  gzdbg << "World == link == inertial frames, with offset" << std::endl;
  model->SetLinkWorldPose(ignition::math::Pose3d::Zero, link);
  EXPECT_EQ(ignition::math::Pose3d::Zero, link->WorldPose());
  EXPECT_EQ(ignition::math::Pose3d::Zero, link->WorldInertialPose());
  this->AddLinkForceTwoWays(_physicsEngine, true, true, world, link,
                            ignition::math::Vector3d(5, 4, 3),
                            ignition::math::Vector3d(-2, 1, 0));

  gzdbg << "World == link != inertial frames, no offset" << std::endl;
  model->SetLinkWorldPose(ignition::math::Pose3d::Zero, link);
  ignition::math::Pose3d inertialPose = ignition::math::Pose3d(
      ignition::math::Vector3d(1, 5, 8),
      ignition::math::Quaterniond(IGN_PI/3.0, IGN_PI*1.5, IGN_PI/4));
  link->GetInertial()->SetCoG(inertialPose);
  link->UpdateMass();
  EXPECT_EQ(ignition::math::Pose3d::Zero, link->WorldPose());
  EXPECT_EQ(inertialPose, link->WorldInertialPose());
  this->AddLinkForceTwoWays(_physicsEngine, true, false, world, link,
                            ignition::math::Vector3d(1, 2, 1));

  gzdbg << "World != link != inertial frames, with offset" << std::endl;
  model->SetLinkWorldPose(ignition::math::Pose3d(
        ignition::math::Vector3d(5, 10, -4),
        ignition::math::Quaterniond(0, IGN_PI/2.0, IGN_PI/6)), link);
  inertialPose = ignition::math::Pose3d(ignition::math::Vector3d(0, -5, 10),
      ignition::math::Quaterniond(0, 2.0*IGN_PI, IGN_PI/3));
  link->GetInertial()->SetCoG(inertialPose);
  link->UpdateMass();
  this->AddLinkForceTwoWays(_physicsEngine, false, false, world, link,
                            ignition::math::Vector3d(1, 2, 1),
                            ignition::math::Vector3d(-2, 0.5, 1));

  gzdbg << "World != link != inertial frames, with offset and initial vel"
      << std::endl;
  model->SetLinkWorldPose(ignition::math::Pose3d(
        ignition::math::Vector3d(-1.5, 0.8, 3),
        ignition::math::Quaterniond(-IGN_PI/4.5, IGN_PI/3.0, IGN_PI*1.2)),
        link);
  inertialPose = ignition::math::Pose3d(ignition::math::Vector3d(1, 0, -5.6),
      ignition::math::Quaterniond(IGN_PI/9, 0, IGN_PI*3));
  link->GetInertial()->SetCoG(inertialPose);
  link->UpdateMass();
  link->SetLinearVel(ignition::math::Vector3d(2, -0.1, 5));
  link->SetAngularVel(ignition::math::Vector3d(-IGN_PI/10, 0, 0.0001));
  this->AddLinkForceTwoWays(_physicsEngine, false, false, world, link,
                            ignition::math::Vector3d(-3, 2.5, -15),
                            ignition::math::Vector3d(-6, -1, -0.2));
}

/////////////////////////////////////////////////
// GetWorldAngularMomentum:
// Spawn box and verify Link::GetWorldAngularMomentum functions
// Make dimensions unequal and give angular velocity that causes
// gyroscopic tumbling.
void PhysicsLinkTest::GetWorldAngularMomentum(const std::string &_physicsEngine)
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  auto physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  world->SetGravity(ignition::math::Vector3d::Zero);

  physics::ModelPtr model;
  {
    // Box size
    const double dx = 0.1;
    const double dy = 0.4;
    const double dz = 0.9;
    const double mass = 10.0;

    msgs::Model msgModel;
    msgModel.set_name(this->GetUniqueString("model"));
    msgs::AddBoxLink(msgModel, mass, ignition::math::Vector3d(dx, dy, dz));
    model = this->SpawnModel(msgModel);
  }
  ASSERT_TRUE(model != NULL);

  // inertia matrix, recompute if dimensions change
  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const ignition::math::Matrix3d I0(
      Ixx, 0.0, 0.0 , 0.0, Iyy, 0.0 , 0.0, 0.0, Izz);

  // Since Ixx > Iyy > Izz,
  // angular velocity with large y component
  // will cause gyroscopic tumbling
  const ignition::math::Vector3d w0(1e-3, 1.5e0, 1.5e-2);
  model->SetAngularVel(w0);

  // Get link and verify inertia and initial velocity
  auto link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  ASSERT_EQ(w0, link->WorldAngularVel());
  ASSERT_EQ(I0, link->WorldInertiaMatrix());

  // Compute initial angular momentum
  const auto H0((I0 * w0));
  ASSERT_EQ(H0, link->WorldAngularMomentum());
  const double H0mag = H0.Length();

  ignition::math::Vector3Stats angularMomentumError;
  const std::string stat("maxAbs");
  EXPECT_TRUE(angularMomentumError.InsertStatistic(stat));
  const int steps = 5000;
  for (int i = 0; i < steps; ++i)
  {
    world->Step(1);
    auto H = link->WorldAngularMomentum();
    angularMomentumError.InsertData((H - H0) / H0mag);
  }
  EXPECT_LT(angularMomentumError.Mag().Map()[stat], g_tolerance * 10);

  RecordProperty("engine", _physicsEngine);
  this->Record("angularMomentumError", angularMomentumError);
}

/////////////////////////////////////////////////
void PhysicsLinkTest::GetWorldEnergy(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // Get gravity magnitude
  double g = world->Gravity().Length();

  // Spawn a box
  double z0 = 10.0;
  ignition::math::Vector3d size(1, 1, 1);
  ignition::math::Vector3d pos0(0, 0, z0 + size.Z() / 2);
  SpawnBox("box", size, pos0, ignition::math::Vector3d::Zero, false);
  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Get initial energy
  double energy0 = link->GetWorldEnergy();
  EXPECT_NEAR(link->GetWorldEnergyKinetic(), 0, g_tolerance);

  double totalTime = sqrt(2*z0/g)*0.95;
  unsigned int stepSize = 10;
  unsigned int steps = floor(totalTime / (dt*stepSize));
  for (unsigned int i = 0; i < steps; ++i)
  {
    world->Step(stepSize);
    double energy = link->GetWorldEnergy();
    EXPECT_NEAR(energy / energy0, 1.0, g_tolerance * 10);
  }
}

/////////////////////////////////////////////////
// GetWorldInertia:
// Spawn boxes and verify Link::GetWorldInertia* functions
void PhysicsLinkTest::GetWorldInertia(const std::string &_physicsEngine)
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  auto physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Box size
  const double dx = 1.0;
  const double dy = 4.0;
  const double dz = 9.0;
  const double mass = 10.0;
  const double angle = IGN_PI / 3.0;

  const unsigned int testCases = 5;
  for (unsigned int i = 0; i < testCases; ++i)
  {
    // Use msgs::AddBoxLink
    msgs::Model msgModel;
    ignition::math::Pose3d modelPose, linkPose, inertialPose;

    msgModel.set_name(this->GetUniqueString("model"));
    msgs::AddBoxLink(msgModel, mass, ignition::math::Vector3d(dx, dy, dz));
    modelPose.Pos().X() = i * dz;
    modelPose.Pos().Z() = dz;

    // i=0: rotated model pose
    //  expect inertial pose to match model pose
    if (i == 0)
    {
      modelPose.Rot().Euler(0.0, 0.0, angle);
    }
    // i=1: rotated link pose
    //  expect inertial pose to match link pose
    else if (i == 1)
    {
      linkPose.Rot().Euler(0.0, 0.0, angle);
    }
    // i=2: rotated inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 2)
    {
      inertialPose.Rot().Euler(0.0, 0.0, angle);
    }
    // i=3: off-diagonal terms in inertia matrix
    //  expect inertial pose to differ from link pose
    else if (i == 3)
    {
      ignition::math::MassMatrix3d m;
      EXPECT_TRUE(m.SetFromBox(mass, ignition::math::Vector3d(dx, dy, dz),
          ignition::math::Quaterniond(0, 0, angle)));
      msgs::Set(msgModel.mutable_link(0)->mutable_inertial(), m);
    }
    // i=4: offset inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 4)
    {
      inertialPose.Pos().Set(1, 1, 1);
    }

    {
      auto msgLink = msgModel.mutable_link(0);
      auto msgInertial = msgLink->mutable_inertial();

      msgs::Set(msgModel.mutable_pose(), modelPose);
      msgs::Set(msgLink->mutable_pose(), linkPose);
      msgs::Set(msgInertial->mutable_pose(), inertialPose);
    }

    auto model = this->SpawnModel(msgModel);
    ASSERT_TRUE(model != NULL);

    auto link = model->GetLink();
    ASSERT_TRUE(link != NULL);

    EXPECT_EQ(model->WorldPose(), modelPose);
    EXPECT_EQ(link->WorldPose(), linkPose + modelPose);
    // only check inertial position
    // inertial rotation can vary (bullet for example)
    EXPECT_EQ(link->WorldInertialPose().Pos(),
              (inertialPose + linkPose + modelPose).Pos());

    // i=0: rotated model pose
    //  expect inertial pose to match model pose
    if (i == 0)
    {
      EXPECT_EQ(model->WorldPose(), link->WorldInertialPose());
    }
    // i=1: rotated link pose
    //  expect inertial pose to match link pose
    else if (i == 1)
    {
      EXPECT_EQ(link->WorldPose(), link->WorldInertialPose());
    }
    // i=2: rotated inertial pose
    // i=3: off-diagonal inertia matrix terms
    //  expect center-of-mass positions to match
    else if (i == 2 || i == 3)
    {
      EXPECT_EQ(link->WorldPose().Pos(),
                link->WorldInertialPose().Pos());
    }
    // i=4: offset inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 4)
    {
      EXPECT_EQ(link->WorldPose().Pos() + inertialPose.Pos(),
                link->WorldInertialPose().Pos());
    }

    // Expect rotated inertia matrix
    ignition::math::Matrix3d inertia = link->WorldInertiaMatrix();
    if (i == 4)
    {
      EXPECT_NEAR(inertia(0, 0), 80.8333, 1e-4);
      EXPECT_NEAR(inertia(1, 1), 68.3333, 1e-4);
      EXPECT_NEAR(inertia(2, 2), 14.1667, 1e-4);
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          if (row != col)
            EXPECT_NEAR(inertia(row, col), 0.0, g_tolerance);
    }
    else
    {
      EXPECT_NEAR(inertia(0, 0), 71.4583, 1e-4);
      EXPECT_NEAR(inertia(1, 1), 77.7083, 1e-4);
      EXPECT_NEAR(inertia(2, 2), 14.1667, 1e-4);
      EXPECT_NEAR(inertia(0, 1),  5.4126, 1e-4);
      EXPECT_NEAR(inertia(1, 0),  5.4126, 1e-4);
      EXPECT_NEAR(inertia(0, 2), 0, g_tolerance);
      EXPECT_NEAR(inertia(2, 0), 0, g_tolerance);
      EXPECT_NEAR(inertia(1, 2), 0, g_tolerance);
      EXPECT_NEAR(inertia(2, 1), 0, g_tolerance);
    }

    // For 0-3, apply torque and expect equivalent response
    if (i <= 3)
    {
      for (int step = 0; step < 50; ++step)
      {
        link->SetTorque(ignition::math::Vector3d(100, 0, 0));
        world->Step(1);
      }

      ignition::math::Vector3d vel = link->WorldAngularVel();
      EXPECT_NEAR(vel.X(),  0.0703, g_tolerance);
      EXPECT_NEAR(vel.Y(), -0.0049, g_tolerance);
      EXPECT_NEAR(vel.Z(),  0.0000, g_tolerance);
    }
  }
}

/////////////////////////////////////////////////
void PhysicsLinkTest::OnWrenchMsg(const std::string &_physicsEngine)
{
  // TODO bullet and simbody currently fail this test
  if (_physicsEngine != "ode" && _physicsEngine != "dart")
  {
    gzerr << "Aborting OnWrenchMsg test for Bullet and Simbody. "
          << "Because of issues #1476 and #1478." << std::endl;
    return;
  }

  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // disable gravity
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Spawn a box
  ignition::math::Vector3d size(1, 1, 1);
  SpawnBox("box", size, ignition::math::Vector3d::Zero,
                        ignition::math::Vector3d::Zero, false);
  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Check that link is at rest
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearAccel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularAccel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldForce());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldTorque());

  // Publish wrench message
  std::string topicName = "~/" + link->GetScopedName() + "/wrench";
  boost::replace_all(topicName, "::", "/");
  transport::PublisherPtr wrenchPub =
    this->node->Advertise<msgs::Wrench>(topicName);

  msgs::Wrench msg;

  std::vector<ignition::math::Vector3d> forces;
  std::vector<ignition::math::Vector3d> torques;
  std::vector<ignition::math::Vector3d> forceOffsets;

  // Only force
  forces.push_back(ignition::math::Vector3d(1, 0, 0));
  torques.push_back(ignition::math::Vector3d::Zero);
  forceOffsets.push_back(ignition::math::Vector3d::Zero);

  // Only force, with an offset
  forces.push_back(ignition::math::Vector3d(5.2, 0.1, 10));
  torques.push_back(ignition::math::Vector3d::Zero);
  forceOffsets.push_back(ignition::math::Vector3d(2.1, 1, -0.6));

  // Only torque
  forces.push_back(ignition::math::Vector3d::Zero);
  torques.push_back(ignition::math::Vector3d(-0.2, 5, 0));
  forceOffsets.push_back(ignition::math::Vector3d::Zero);

  // All fields set
  forces.push_back(ignition::math::Vector3d(5, 6, -0.9));
  torques.push_back(ignition::math::Vector3d(-0.2, 5, 0));
  forceOffsets.push_back(ignition::math::Vector3d(-1, -4, -0.8));

  // When DART is the phyiscs engine, in order to pass this test, we need to
  // turn off the default behavior of clearing the forces and torques after
  // each simulation step. The force and torques that are sent over messages
  // don't get received by the simulation engine until the update step occurs,
  // but the update step will also simulate forward, and DART's default behavior
  // would then clear out the forces and torques. Setting this parameter
  // overrides that behavior.
  if ("dart" == _physicsEngine)
  {
    physics->SetParam("auto_reset_forces", false);
  }

  for (unsigned int i = 0; i < forces.size(); ++i)
  {
    gzdbg << "Testing force: " << forces[i].X() << ", "
                               << forces[i].Y() << ", "
                               << forces[i].Z() <<
                   " torque: " << torques[i].X() << ", "
                               << torques[i].Y() << ", "
                               << torques[i].Z() <<
             " force offset: " << forceOffsets[i].X() << ", "
                               << forceOffsets[i].Y() << ", "
                               << forceOffsets[i].Z() << std::endl;

    // Publish message
    msgs::Set(msg.mutable_force(), forces[i]);
    msgs::Set(msg.mutable_torque(), torques[i]);
    // Leave optional field unset if it's zero
    if (forceOffsets[i] != ignition::math::Vector3d::Zero)
      msgs::Set(msg.mutable_force_offset(), forceOffsets[i]);

    wrenchPub->Publish(msg);

    // Calculate expected values
    ignition::math::Vector3d forceWorld = forces[i];
    ignition::math::Vector3d worldOffset =
      forceOffsets[i] - link->GetInertial()->CoG();
    ignition::math::Vector3d torqueWorld =
      worldOffset.Cross(forces[i]) + torques[i];

    double tolerance = 1e-6;
    // Note: Similar to the issue in AddLinkForceTwoWays, DART experiences some
    // inaccuracies when returning the WorldForce. I suspect this is because of
    // numerical inaccuracies built up through matrix multiplications. Perhaps
    // we should check on the accuracy of the matrix quantities that we provide
    // to DART.
    if ("dart" == _physicsEngine)
    {
      tolerance = 2e-3;
    }

    // Wait for message to be received
    while (!link->WorldForce().Equal(forceWorld, tolerance) ||
           !link->WorldTorque().Equal(torqueWorld, 1e-6) )
    {
      world->Step(1);
      common::Time::MSleep(1);
    }

    // Check force and torque (at CoG?) in world frame
    VEC_EXPECT_NEAR(link->WorldForce(), forceWorld, tolerance);
    VEC_EXPECT_NEAR(link->WorldTorque(), torqueWorld, 1e-6);

    // Reset link's physics states
    link->ResetPhysicsStates();
    link->SetWorldPose(ignition::math::Pose3d());
    world->Step(1);

    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearVel());
    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());
    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearAccel());
    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularAccel());
    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldForce());
    EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldTorque());
  }
}

/////////////////////////////////////////////////
void PhysicsLinkTest::SetVelocity(const std::string &_physicsEngine)
{
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // disable gravity
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Spawn a box
  ignition::math::Vector3d size(1, 1, 1);
  ignition::math::Vector3d pos0(0, 0, 1);
  SpawnBox("box", size, pos0, ignition::math::Vector3d::Zero, false);
  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Set upward velocity and check
  ignition::math::Vector3d vel(0, 0, 1);
  link->SetLinearVel(vel);
  world->Step(1);
  EXPECT_EQ(vel, link->WorldLinearVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());

  // Step forward and check velocity again
  world->Step(44);
  double time = world->SimTime().Double();
  EXPECT_EQ(vel, link->WorldLinearVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());

  // check position
  ignition::math::Vector3d pos = link->WorldPose().Pos();
  EXPECT_EQ(pos0 + time*vel, pos);

  // Set velocity to zero
  link->SetLinearVel(ignition::math::Vector3d::Zero);
  world->Step(1);
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldLinearVel());
  EXPECT_EQ(ignition::math::Vector3d::Zero, link->WorldAngularVel());
  EXPECT_EQ(pos0 + time*vel, pos);

  // Start translating and rotating
  vel.Set(1, 1, 0);
  ignition::math::Vector3d vel2(0, 2.0, 0);
  link->SetLinearVel(vel);
  link->SetAngularVel(vel2);

  // Step once
  world->Step(1);
  EXPECT_EQ(vel, link->WorldLinearVel());
  EXPECT_EQ(vel2, link->WorldAngularVel());

  // test linear velocity at specific point in space
  ignition::math::Vector3d offset(0, 0, -0.5);
  ignition::math::Vector3d vel3 = link->WorldLinearVel(offset,
      ignition::math::Quaterniond::Identity);
  EXPECT_NEAR(vel3.X(), 0.0, g_tolerance);
  EXPECT_NEAR(vel3.Y(), 1.0, g_tolerance);
  EXPECT_NEAR(vel3.Z(), 0.0, g_tolerance);

  // check rotation
  ignition::math::Vector3d rpy = link->WorldPose().Rot().Euler();
  EXPECT_NEAR(rpy.X(), 0.0, g_tolerance);
  EXPECT_NEAR(rpy.Y(), vel2.Y()*dt, g_tolerance);
  EXPECT_NEAR(rpy.Z(), 0.0, g_tolerance);
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, AddForce)
{
  AddForce(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, GetWorldAngularMomentum)
{
  GetWorldAngularMomentum(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, GetWorldEnergy)
{
  GetWorldEnergy(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, GetWorldInertia)
{
  GetWorldInertia(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, OnWrenchMsg)
{
  OnWrenchMsg(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, SetVelocity)
{
  SetVelocity(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsLinkTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
