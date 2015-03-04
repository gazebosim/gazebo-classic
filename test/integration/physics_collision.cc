/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;
const double g_big = 1e29;
const double g_physics_tol = 1e-2;

class PhysicsCollisionTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Test Collision::GetBoundingBox.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetBoundingBox(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsCollisionTest::GetBoundingBox(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody" ||
      _physicsEngine == "dart")
  {
    gzerr << "Bounding boxes not yet working with "
          << _physicsEngine
          << ", see issue #1148"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check bounding box of ground plane
  {
    physics::ModelPtr model = world->GetModel("ground_plane");
    math::Box box = model->GetBoundingBox();
    EXPECT_LT(box.min.x, -g_big);
    EXPECT_LT(box.min.y, -g_big);
    EXPECT_LT(box.min.z, -g_big);
    EXPECT_GT(box.max.x, g_big);
    EXPECT_GT(box.max.y, g_big);
    EXPECT_DOUBLE_EQ(box.max.z, 0.0);
  }
}

/////////////////////////////////////////////////
TEST_F(PhysicsCollisionTest, ModelSelfCollide)
{
  // self_collide is only implemented in ODE
  Load("worlds/model_self_collide.world", true, "ode");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  math::Vector3 g = physics->GetGravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // 3 models: all_collide, some_collide, and no_collide
  physics::ModelPtr all_collide, some_collide, no_collide;
  all_collide = world->GetModel("all_collide");
  some_collide = world->GetModel("some_collide");
  no_collide = world->GetModel("no_collide");
  ASSERT_TRUE(all_collide != NULL);
  ASSERT_TRUE(some_collide != NULL);
  ASSERT_TRUE(no_collide != NULL);

  // Step forward 0.2 s
  double stepTime = 0.2;
  unsigned int steps = floor(stepTime / dt);
  world->Step(steps);

  // Expect boxes to be falling
  double fallVelocity = g.z * stepTime;
  EXPECT_LT(all_collide->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));
  EXPECT_LT(some_collide->GetWorldLinearVel().z,
      fallVelocity*(1-g_physics_tol));
  EXPECT_LT(no_collide->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));

  // Another 3000 steps should put the boxes at rest
  world->Step(3000);

  // Expect 3 boxes to be stationary
  EXPECT_NEAR(all_collide->GetWorldLinearVel().z, 0, 1e-2);
  EXPECT_NEAR(some_collide->GetWorldLinearVel().z, 0, 1e-2);
  EXPECT_NEAR(no_collide->GetWorldLinearVel().z, 0, 1e-2);

  // link2 of all_collide should have the highest z-coordinate (around 3)
  EXPECT_NEAR(all_collide->GetLink("link2")->GetWorldPose().pos.z, 2.5, 1e-2);

  // link2 of some_collide should have a middling z-coordinate (around 2)
  EXPECT_NEAR(some_collide->GetLink("link2")->GetWorldPose().pos.z, 1.5, 1e-2);

  // link2 of no_collide should have a low z-coordinate (around 1)
  EXPECT_NEAR(no_collide->GetLink("link2")->GetWorldPose().pos.z, 0.5, 1e-2);

  Unload();
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, GetBoundingBox)
{
  GetBoundingBox(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsCollisionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
