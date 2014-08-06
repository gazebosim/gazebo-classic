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
  if (_physicsEngine == "dart")
  {
    gzerr << "DARTLink::SetLinearVel, SetAngularVel aren't working (#1079)"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
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
TEST_P(PhysicsLinkTest, GetWorldEnergy)
{
  GetWorldEnergy(GetParam());
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
