/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <map>
#include <string>
#include <vector>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsTest : public ServerFixture,
  public testing::WithParamInterface<const char*>
{
  public: void DropTest(const std::string &_physicsEngine,
              const std::string &_solverType,
              const std::string &_worldSolverType);
};

////////////////////////////////////////////////////////////////////////
void PhysicsTest::DropTest(const std::string &_physicsEngine,
    const std::string &_solverType,
    const std::string &_worldSolverType)
{
  Load("worlds/drop_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // check the gravity vector
  auto gravity = world->Gravity();
  EXPECT_DOUBLE_EQ(gravity.X(), 0);
  EXPECT_DOUBLE_EQ(gravity.Y(), 0);
  EXPECT_DOUBLE_EQ(gravity.Z(), -10.0);

  if (_physicsEngine == "ode")
  {
    // Set solver type
    physics->SetParam("solver_type", _solverType);
    // Set world step solver type
    physics->SetParam("world_step_solver", _worldSolverType);
  }
  math::Pose pose;
  physics::ModelPtr sphere_model = world->GetModel("sphere");
  if (sphere_model)
    pose = sphere_model->GetWorldPose();

  double z = pose.pos.z;
  double test_duration = 3.0;

  // Dynamic duration includes the bounce back
  double dynamic_duration = 2.4;
  double v = 0.0;
  double g = -10.0;
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  int steps = test_duration/dt;
  int dynamic_steps = dynamic_duration/dt;
  for (int i = 0; i < steps; ++i)
  {
    v += dt * g;
    z += dt * v;

    world->Step(1);
    physics::ModelPtr sphere_model = world->GetModel("sphere");
    if (sphere_model)
    {
      math::Vector3 vel = sphere_model->GetWorldLinearVel();
      math::Pose pose = sphere_model->GetWorldPose();
      if (z > 0.5)
      {
        EXPECT_LT(fabs(vel.z - v), PHYSICS_TOL);
        EXPECT_LT(fabs(pose.pos.z - z), PHYSICS_TOL);
      }

      // After contact with ground, and no bounce back
      if (i > dynamic_steps)
      {
        if (_worldSolverType == "BULLET_LEMKE")
        {
          gzerr << "This test isn't working well during contact with "
                << _worldSolverType
                << ", see issue #1846"
                << std::endl;
          break;
        }
        EXPECT_LT(fabs(vel.z), PHYSICS_TOL);
        EXPECT_LT(fabs(pose.pos.z - 0.5), PHYSICS_TOL);
      }
    }
  }
}

TEST_P(PhysicsTest, DropTest)
{
  gzdbg << "DropTest(ode, world, " << GetParam() << ')' << std::endl;
  DropTest("ode", "world", GetParam());
}

INSTANTIATE_TEST_CASE_P(WorldStepSolvers, PhysicsTest,
                        WORLD_STEP_SOLVERS);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
