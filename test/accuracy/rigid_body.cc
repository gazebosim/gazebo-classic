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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

class RigidBodyTest : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void OneBox(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
// OneBox:
// Spawn a single box and record accuracy for momentum and enery
// conservation
void RigidBodyTest::OneBox(const std::string &_physicsEngine)
{
  // Load a blank world (no ground plane)
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get gravity value
  math::Vector3 g = physics->GetGravity();

  // Box size
  const double dx = 1.0;
  const double dy = 4.0;
  const double dz = 9.0;
  const double mass = 10.0;

  // Create box with inertia based on box of uniform density
  msgs::Model msgModel;
  msgModel.set_name(this->GetUniqueString("model"));
  msgs::AddBoxLink(msgModel, mass, math::Vector3(dx, dy, dz));

  physics::ModelPtr model = this->SpawnModel(msgModel);
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // Give initial impulse to set initial conditions
  link->SetForce(math::Vector3(1e0, 1e1, 1e2));
  link->SetTorque(math::Vector3(1e4, 1e3, 1e2));
  world->Step(1);

  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear velocity in global frame
  math::Vector3 v0 = link->GetWorldAngularVel();

  // initial angular momentum in global frame
  math::Vector3 H0 = link->GetWorldInertiaMatrix() * link->GetWorldAngularVel();
  double H0mag = H0.GetLength();

  // initial energy
  double E0 = link->GetWorldEnergy();

  // variables to compute statistics on
  math::Vector3Stats linearVelocityError;
  math::Vector3Stats angularMomentumError;
  math::SignalStats energyError;
  {
    const std::string statNames = "MaxAbs,Rms";
    EXPECT_TRUE(linearVelocityError.InsertStatistics(statNames));
    EXPECT_TRUE(angularMomentumError.InsertStatistics(statNames));
    EXPECT_TRUE(energyError.InsertStatistics(statNames));
  }

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < 15000; ++i)
  {
    world->Step(1);

    // angular momentum error
    math::Vector3 H = link->GetWorldInertiaMatrix() * link->GetWorldAngularVel();
    angularMomentumError.InsertData((H - H0) / H0mag);

    // energy error
    gzerr << (link->GetWorldEnergy() - E0) / E0 << std::endl;
    energyError.InsertData((link->GetWorldEnergy() - E0) / E0);
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("elapsedWallTime", elapsedTime.Double());
  this->Record("simTime", world->GetSimTime().Double());

  // Record statistics on pitch and yaw angles
  this->Record("energyError", energyError);
  this->Record("angMomentumErr", angularMomentumError.mag);
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, OneBox)
{
  OneBox(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, RigidBodyTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
