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

#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;

class PhysicsLinkTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Test GetWorldEnergy* functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetWorldEnergy(const std::string &_physicsEngine);

  /// \brief Test velocity setting functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetVelocity(const std::string &_physicsEngine);

  /// \brief Test force adding functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void AddForce(const std::string &_physicsEngine);
};

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
void PhysicsLinkTest::AddForce(const std::string &_physicsEngine)
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
  math::Vector3 pos0(0, 0, 0);
  math::Vector3 rot0(0, 0, 0);
  math::Quaternion quat0(rot0);
  SpawnBox("box", size, pos0, rot0, false);
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  double mass = link->GetInertial()->GetMass();
  EXPECT_EQ(1.0, mass);
  EXPECT_EQ(math::Pose(math::Vector3::Zero, math::Vector3::Zero),
      link->GetWorldInertialPose());

  // Check that link is at rest
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  int largeNumerOfSteps = 44;

  /////////////////////////////////////////////////
  // Force at link frame
  // CASE 1
  // Apply upwards (World Z+ == Link Z+) at link/world origin
  // causing no rotation
  math::Vector3 force(0, 0, 1);
  link->AddLinkForce(force);
  world->Step(1);
  math::Vector3 newLinearAccel = force/mass;
  math::Vector3 newLinearVel = dt*force/mass;

  // Check acceleration in world frame
  EXPECT_EQ(newLinearAccel, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check velocity in world frame
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // Step forward and check again
  world->Step(largeNumerOfSteps);

  // Check that acceleration is zero
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check that velocity hasn't changed
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // Add opposing force in link frame and check that link returned to initial
  // pose and stopped
  force = math::Vector3(0, 0, -1);
  link->AddLinkForce(force);
  world->Step(largeNumerOfSteps);
  EXPECT_EQ(pos0, link->GetWorldPose().pos);
  EXPECT_EQ(quat0, link->GetWorldPose().rot);
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // CASE 2
  // Dislocate model so that World Z+ == Link X- and there's an offset
  pos0 = math::Vector3(2, 0, 0);
  rot0 = math::Vector3(0, M_PI/2.0, 0);
  quat0 = math::Quaternion(rot0);
  model->SetLinkWorldPose(math::Pose(pos0, rot0), link);

  // Apply upwards (World Z+ == Link X-) at link origin
  // causing no rotation
  force = math::Vector3(-1, 0, 0);
  link->AddLinkForce(force);
  world->Step(1);

  // Check acceleration in world frame
  EXPECT_EQ(newLinearAccel, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check velocity in world frame
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // Step forward and check again
  world->Step(largeNumerOfSteps);

  // Check that acceleration is zero
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check that velocity hasn't changed
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());

  // Add opposing force in link frame and check that link returned to initial
  // pose and stopped
  force = math::Vector3(1, 0, 0);
  link->AddLinkForce(force);
  world->Step(largeNumerOfSteps);
  EXPECT_EQ(pos0, link->GetWorldPose().pos);
  EXPECT_EQ(quat0, link->GetWorldPose().rot);
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // CASE 3
  // Apply upwards (World Z+ == Link X-) at
  // world point (1, 0, 0) == link point (0, 0, -1)
  // causing rotation about world Y+
  force = math::Vector3(-1, 0, 0);
  math::Vector3 forcePos = math::Vector3(0, 0, -1);
  link->AddLinkForce(force, forcePos);
  world->Step(1);
//  double radius = forcePos.GetLength();
//  math::Vector3 newAngularAccel = newLinearAccel/radius;
//  math::Vector3 newAngularVel = newLinearVel/radius;

  // Check acceleration in world frame
  EXPECT_EQ(newLinearAccel, link->GetWorldLinearAccel());
//  EXPECT_EQ(newAngularAccel, link->GetWorldAngularAccel());
  EXPECT_NEAR(0, link->GetWorldAngularAccel().x, g_tolerance);
  EXPECT_LT(0, link->GetWorldAngularAccel().y);
  EXPECT_NEAR(0, link->GetWorldAngularAccel().z, g_tolerance);

  // Check velocity in world frame
  math::Vector3 newAngularVel = link->GetWorldAngularVel();
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_NEAR(0, newAngularVel.x, g_tolerance);
  EXPECT_LT(0, newAngularVel.y);
  EXPECT_NEAR(0, newAngularVel.z, g_tolerance);

  // Step forward and check again
  world->Step(largeNumerOfSteps);

  // Check that acceleration is zero
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check that velocity hasn't changed
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(newAngularVel, link->GetWorldAngularVel());

  // Add opposing force in link frame and check that link returned to initial
  // pose and stopped
  force = math::Vector3(1, 0, 0);
  forcePos = math::Vector3(0, 0, -1);
  link->AddLinkForce(force, forcePos);
  world->Step(largeNumerOfSteps);
  EXPECT_EQ(pos0, link->GetWorldPose().pos);
  EXPECT_EQ(quat0, link->GetWorldPose().rot);
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // CASE 4
  // Dislocate CoG to make sure we're acting on link frame, not inertial frame
  pos0 = math::Vector3::Zero;
  rot0 = math::Vector3::Zero;
  quat0 = math::Quaternion(rot0);
  model->SetLinkWorldPose(math::Pose(pos0, rot0), link);

  math::Pose inertialPose = math::Pose(math::Vector3(1, 0, 0),
      math::Vector3(0, 0, M_PI/2.0));
  link->GetInertial()->SetCoG(inertialPose);
  // link frame == world frame
  EXPECT_EQ(inertialPose, link->GetWorldInertialPose());

  // Apply upwards (World Z+ == Link Z+) at link origin
  // causing rotation about world Y+
  force = math::Vector3(0, 0, 1);
  link->AddLinkForce(force);
  world->Step(1);

  // Check acceleration in world frame
  EXPECT_EQ(newLinearAccel, link->GetWorldLinearAccel());
  EXPECT_NEAR(0, link->GetWorldAngularAccel().x, g_tolerance);
  EXPECT_LT(0, link->GetWorldAngularAccel().y);
  EXPECT_NEAR(0, link->GetWorldAngularAccel().z, g_tolerance);

  // Check velocity in world frame
  newAngularVel = link->GetWorldAngularVel();
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_NEAR(0, newAngularVel.x, g_tolerance);
  EXPECT_LT(0, newAngularVel.y);
  EXPECT_NEAR(0, newAngularVel.z, g_tolerance);

  // Step forward and check again
  world->Step(largeNumerOfSteps);

  // Check that acceleration is zero
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());

  // Check that velocity hasn't changed
  EXPECT_EQ(newLinearVel, link->GetWorldLinearVel());
  EXPECT_EQ(newAngularVel, link->GetWorldAngularVel());

  // Add opposing force in link frame and check that link returned to initial
  // pose and stopped
  force = math::Vector3(0, 0, -1);
  link->AddLinkForce(force);
  world->Step(largeNumerOfSteps);
  EXPECT_EQ(pos0, link->GetWorldPose().pos);
  EXPECT_EQ(quat0, link->GetWorldPose().rot);
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularVel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldLinearAccel());
  EXPECT_EQ(math::Vector3::Zero, link->GetWorldAngularAccel());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, GetWorldEnergy)
{
  GetWorldEnergy(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, SetVelocity)
{
  SetVelocity(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsLinkTest, AddForce)
{
  AddForce(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsLinkTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
