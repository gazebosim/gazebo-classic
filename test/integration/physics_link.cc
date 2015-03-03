/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;

class PhysicsLinkTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Test force adding functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void AddForce(const std::string &_physicsEngine);

  /// \brief Use AddLinkForce on the given direction and then the opposite
  /// direction so they cancel out.
  /// \param[in] _world World pointer.
  /// \param[in] _link Link pointer.
  /// \param[in] _force Force expressed in link frame.
  /// \param[in] _offset Offset expressed in link frame, defaults to link
  /// origin.
  public: void AddLinkForceTwoWays(physics::WorldPtr _world,
      physics::LinkPtr _link, math::Vector3 _force,
      math::Vector3 _offset = math::Vector3::Zero);

  /// \brief Test GetWorldEnergy* functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetWorldEnergy(const std::string &_physicsEngine);

  /// \brief Test Link::GetWorldInertia* functions.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void LinkGetWorldInertia(const std::string &_physicsEngine);

  /// \brief Test velocity setting functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetVelocity(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsLinkTest::AddLinkForceTwoWays(physics::WorldPtr _world,
    physics::LinkPtr _link, math::Vector3 _force,
    math::Vector3 _offset)
{
  // Get state before adding force
  math::Vector3 linearVelWorld0 = _link->GetWorldCoGLinearVel();
  math::Vector3 angularVelWorld0 = _link->GetWorldAngularVel();
  math::Pose poseWorld0 = _link->GetWorldPose();

  // Add Link Force
  if (_offset == math::Vector3::Zero)
    _link->AddLinkForce(_force);
  else
    _link->AddLinkForce(_force, _offset);

  double dt = _world->GetPhysicsEngine()->GetMaxStepSize();
  _world->Step(1);

  int moreThanOneStep = 2;

  // Check force and torque (at CoG?) in world frame
  math::Vector3 forceWorld = poseWorld0.rot.RotateVector(_force);
  EXPECT_EQ(forceWorld, _link->GetWorldForce());

  math::Vector3 worldOffset = poseWorld0.rot.RotateVector(
      _offset - _link->GetInertial()->GetCoG());
  math::Vector3 angularImpulse = dt*worldOffset.Cross(forceWorld);
  EXPECT_EQ(angularImpulse, _link->GetWorldTorque());

  // Check acceleration in world frame
  math::Vector3 oneStepLinearAccel =
      forceWorld/_link->GetInertial()->GetMass();
  EXPECT_EQ(oneStepLinearAccel, _link->GetWorldLinearAccel());

  math::Vector3 oneStepAngularAccel = angularImpulse;
  EXPECT_EQ(oneStepAngularAccel, _link->GetWorldAngularAccel());

  // Check velocity in world frame
  math::Vector3 oneStepLinearVel = linearVelWorld0 +
      dt*forceWorld/_link->GetInertial()->GetMass();
  EXPECT_EQ(oneStepLinearVel, _link->GetWorldCoGLinearVel());

  math::Vector3 oneStepAngularVel = angularVelWorld0 + angularImpulse /
      _link->GetInertial()->GetPrincipalMoments();
  EXPECT_EQ(oneStepAngularVel, _link->GetWorldAngularVel());

  // Step forward and check again
  _world->Step(moreThanOneStep);

  // Check that force and torque are zero
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldForce());
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldTorque());

  // Check that acceleration is zero
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldAngularAccel());

  // Check that velocity hasn't changed
  EXPECT_EQ(oneStepLinearVel, _link->GetWorldCoGLinearVel());
  EXPECT_EQ(oneStepAngularVel, _link->GetWorldAngularVel());

  // Add opposing force in link frame and check that link is back to initial
  // velocity
  if (_offset == math::Vector3::Zero)
    _link->AddLinkForce(-_force);
  else
    _link->AddLinkForce(-_force, _offset);

  _world->Step(moreThanOneStep);
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldForce());
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldTorque());
  EXPECT_EQ(linearVelWorld0, _link->GetWorldCoGLinearVel());
  EXPECT_EQ(angularVelWorld0, _link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, _link->GetWorldAngularAccel());
}

/////////////////////////////////////////////////
void PhysicsLinkTest::AddForce(const std::string &_physicsEngine)
{
  // TODO bullet, dart and simbody currently fail this test
  if (_physicsEngine != "ode")
  {
    gzerr << "Aborting AddForce test for Bullet, DART and Simbody."
        << std::endl;
    return;
  }

  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  // Spawn a box
  math::Vector3 size(1, 1, 1);
  SpawnBox("box", size, math::Vector3::Zero, math::Vector3::Zero, false);
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Check that link is at rest
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Add force at link frame
  gzdbg << "World == link == inertial frames, no offset" << std::endl;
  EXPECT_EQ(math::Pose(), link->GetWorldPose());
  EXPECT_EQ(math::Pose(), link->GetWorldInertialPose());
  this->AddLinkForceTwoWays(world, link, math::Vector3(1, 20, 31));

  gzdbg << "World != link == inertial frames, no offset" << std::endl;
  model->SetLinkWorldPose(math::Pose(math::Vector3(2, 3, 4),
                          math::Vector3(0, M_PI/2.0, 1)), link);
  EXPECT_NE(math::Pose(), link->GetWorldPose());
  EXPECT_EQ(link->GetWorldPose(), link->GetWorldInertialPose());
  this->AddLinkForceTwoWays(world, link, math::Vector3(-1, 10, 5));

  gzdbg << "World == link == inertial frames, with offset" << std::endl;
  model->SetLinkWorldPose(math::Pose(), link);
  EXPECT_EQ(math::Pose(), link->GetWorldPose());
  EXPECT_EQ(math::Pose(), link->GetWorldInertialPose());
  this->AddLinkForceTwoWays(world, link, math::Vector3(5, 4, 3),
      math::Vector3(-2, 1, 0));

  gzdbg << "World == link != inertial frames, no offset" << std::endl;
  model->SetLinkWorldPose(math::Pose(), link);
  math::Pose inertialPose = math::Pose(math::Vector3(1, 5, 8),
      math::Vector3(M_PI/3.0, M_PI*1.5, M_PI/4));
  link->GetInertial()->SetCoG(inertialPose);
  EXPECT_EQ(math::Pose(), link->GetWorldPose());
  EXPECT_EQ(inertialPose, link->GetWorldInertialPose());
  this->AddLinkForceTwoWays(world, link, math::Vector3(1, 2, 1));

  gzdbg << "World != link != inertial frames, with offset" << std::endl;
  model->SetLinkWorldPose(math::Pose(math::Vector3(5, 10, -4),
                          math::Vector3(0, M_PI/2.0, M_PI/6)), link);
  inertialPose = math::Pose(math::Vector3(0, -5, 10),
      math::Vector3(0, 2.0*M_PI, M_PI/3));
  link->GetInertial()->SetCoG(inertialPose);
  this->AddLinkForceTwoWays(world, link, math::Vector3(1, 2, 1),
      math::Vector3(-2, 0.5, 1));

  gzdbg << "World != link != inertial frames, with offset and initial vel"
      << std::endl;
  model->SetLinkWorldPose(math::Pose(math::Vector3(-1.5, 0.8, 3),
                          math::Vector3(-M_PI/4.5, M_PI/3.0, M_PI*1.2)), link);
  inertialPose = math::Pose(math::Vector3(1, 0, -5.6),
      math::Vector3(M_PI/9, 0, M_PI*3));
  link->GetInertial()->SetCoG(inertialPose);
  link->SetLinearVel(math::Vector3(2, -0.1, 5));
  link->SetAngularVel(math::Vector3(-M_PI/10, 0, 0.0001));
  this->AddLinkForceTwoWays(world, link, math::Vector3(-3, 2.5, -15),
      math::Vector3(-6, -1, -0.2));
}

/////////////////////////////////////////////////
void PhysicsLinkTest::GetWorldEnergy(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // Get gravity magnitude
  double g = physics->GetGravity().GetLength();

  // Spawn a box
  double z0 = 10.0;
  math::Vector3 size(1, 1, 1);
  math::Vector3 pos0(0, 0, z0 + size.z / 2);
  SpawnBox("box", size, pos0, math::Vector3::Zero, false);
  physics::ModelPtr model = world->GetModel("box");
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
// LinkGetWorldInertia:
// Spawn boxes and verify Link::GetWorldInertia* functions
void PhysicsLinkTest::LinkGetWorldInertia(const std::string &_physicsEngine)
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  auto physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  // Box size
  const double dx = 1.0;
  const double dy = 4.0;
  const double dz = 9.0;
  const double mass = 10.0;
  const double angle = M_PI / 3.0;

  const unsigned int testCases = 4;
  for (unsigned int i = 0; i < testCases; ++i)
  {
    // Use msgs::AddBoxLink
    msgs::Model msgModel;
    math::Pose modelPose, linkPose, inertialPose;

    msgModel.set_name(this->GetUniqueString("model"));
    msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));
    modelPose.pos.x = i * dz;
    modelPose.pos.z = dz;

    // i=0: rotated model pose
    //  expect inertial pose to match model pose
    if (i == 0)
    {
      modelPose.rot.SetFromEuler(0.0, 0.0, angle);
    }
    // i=1: rotated link pose
    //  expect inertial pose to match link pose
    else if (i == 1)
    {
      linkPose.rot.SetFromEuler(0.0, 0.0, angle);
    }
    // i=2: rotated inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 2)
    {
      inertialPose.rot.SetFromEuler(0.0, 0.0, angle);
    }
    // i=3: offset inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 3)
    {
      inertialPose.pos.Set(1, 1, 1);
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

    EXPECT_EQ(model->GetWorldPose(), modelPose);
    EXPECT_EQ(link->GetWorldPose(), linkPose + modelPose);
    EXPECT_EQ(link->GetWorldInertialPose(),
              inertialPose + linkPose + modelPose);

    // i=0: rotated model pose
    //  expect inertial pose to match model pose
    if (i == 0)
    {
      EXPECT_EQ(model->GetWorldPose(),
                link->GetWorldInertialPose());
    }
    // i=1: rotated link pose
    //  expect inertial pose to match link pose
    else if (i == 1)
    {
      EXPECT_EQ(link->GetWorldPose(),
                link->GetWorldInertialPose());
    }
    // i=2: offset and rotated inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 2)
    {
      EXPECT_EQ(link->GetWorldPose().pos,
                link->GetWorldInertialPose().pos);
    }
    // i=3: offset inertial pose
    //  expect inertial pose to differ from link pose
    else if (i == 3)
    {
      EXPECT_EQ(link->GetWorldPose().pos + inertialPose.pos,
                link->GetWorldInertialPose().pos);
    }

    // Expect rotated inertia matrix
    math::Matrix3 inertia = link->GetWorldInertiaMatrix();
    if (i == 3)
    {
      EXPECT_NEAR(inertia[0][0], 80.8333, 1e-4);
      EXPECT_NEAR(inertia[1][1], 68.3333, 1e-4);
      EXPECT_NEAR(inertia[2][2], 14.1667, 1e-4);
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          if (row != col)
            EXPECT_NEAR(inertia[row][col], 0.0, g_tolerance);
    }
    else
    {
      EXPECT_NEAR(inertia[0][0], 71.4583, 1e-4);
      EXPECT_NEAR(inertia[1][1], 77.7083, 1e-4);
      EXPECT_NEAR(inertia[2][2], 14.1667, 1e-4);
      EXPECT_NEAR(inertia[0][1],  5.4126, 1e-4);
      EXPECT_NEAR(inertia[1][0],  5.4126, 1e-4);
      EXPECT_NEAR(inertia[0][2], 0, g_tolerance);
      EXPECT_NEAR(inertia[2][0], 0, g_tolerance);
      EXPECT_NEAR(inertia[1][2], 0, g_tolerance);
      EXPECT_NEAR(inertia[2][1], 0, g_tolerance);
    }

    // For 0-2, apply torque and expect equivalent response
    if (i <= 2)
    {
      for (int step = 0; step < 50; ++step)
      {
        link->SetTorque(math::Vector3(100, 0, 0));
        world->Step(1);
      }
      if (_physicsEngine.compare("dart") == 0)
      {
        gzerr << "Dart fails this portion of the test (#1090)" << std::endl;
      }
      else
      {
        math::Vector3 vel = link->GetWorldAngularVel();
        EXPECT_NEAR(vel.x,  0.0703, g_tolerance);
        EXPECT_NEAR(vel.y, -0.0049, g_tolerance);
        EXPECT_NEAR(vel.z,  0.0000, g_tolerance);
      }
    }
  }
}

/////////////////////////////////////////////////
void PhysicsLinkTest::SetVelocity(const std::string &_physicsEngine)
{
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  // Spawn a box
  math::Vector3 size(1, 1, 1);
  math::Vector3 pos0(0, 0, 1);
  SpawnBox("box", size, pos0, math::Vector3::Zero, false);
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Set upward velocity and check
  math::Vector3 vel(0, 0, 1);
  link->SetLinearVel(vel);
  world->Step(1);
  EXPECT_EQ(vel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // Step forward and check velocity again
  world->Step(44);
  double time = world->GetSimTime().Double();
  EXPECT_EQ(vel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // check position
  math::Vector3 pos = link->GetWorldPose().pos;
  EXPECT_EQ(pos0 + time*vel, pos);

  // Set velocity to zero
  link->SetLinearVel(math::Vector3::Zero);
  world->Step(1);
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(pos0 + time*vel, pos);

  // Start translating and rotating
  vel.Set(1, 1, 0);
  math::Vector3 vel2(0, 2.0, 0);
  link->SetLinearVel(vel);
  link->SetAngularVel(vel2);

  // Step once
  world->Step(1);
  EXPECT_EQ(vel, link->GetWorldLinearVel());
  EXPECT_EQ(vel2, link->GetWorldAngularVel());

  // test linear velocity at specific point in space
  math::Vector3 offset(0, 0, -0.5);
  math::Vector3 vel3 = link->GetWorldLinearVel(offset, math::Quaternion());
  EXPECT_NEAR(vel3.x, 0.0, g_tolerance);
  EXPECT_NEAR(vel3.y, 1.0, g_tolerance);
  EXPECT_NEAR(vel3.z, 0.0, g_tolerance);

  // check rotation
  math::Vector3 rpy = link->GetWorldPose().rot.GetAsEuler();
  EXPECT_NEAR(rpy.x, 0.0, g_tolerance);
  EXPECT_NEAR(rpy.y, vel2.y*dt, g_tolerance);
  EXPECT_NEAR(rpy.z, 0.0, g_tolerance);
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, AddForce)
{
  AddForce(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, GetWorldEnergy)
{
  GetWorldEnergy(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, LinkGetWorldInertia)
{
  LinkGetWorldInertia(GetParam());
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
