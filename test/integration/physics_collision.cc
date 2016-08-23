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
#include <map>
#include <string>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

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

  // 4 models: all_collide, some_collide, no_collide, and explicit_no_collide
  std::map<std::string, physics::ModelPtr> models;
  models["all_collide"]         = physics::ModelPtr();
  models["some_collide"]        = physics::ModelPtr();
  models["no_collide"]          = physics::ModelPtr();
  models["explicit_no_collide"] = physics::ModelPtr();
  for (auto &iter : models)
  {
    gzdbg << "Getting model " << iter.first << std::endl;
    iter.second = world->GetModel(iter.first);
    ASSERT_TRUE(iter.second != NULL);
  }

  // Step forward 0.2 s
  double stepTime = 0.2;
  unsigned int steps = floor(stepTime / dt);
  world->Step(steps);

  // Expect boxes to be falling
  double fallVelocity = g.z * stepTime;
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check falling: " << link->GetScopedName() << std::endl;
      EXPECT_LT(link->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));
    }
  }

  // Another 3000 steps should put the boxes at rest
  world->Step(3000);

  // Expect 3 boxes to be stationary
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check resting: " << link->GetScopedName() << std::endl;
      EXPECT_NEAR(link->GetWorldLinearVel().z, 0, g_physics_tol);
    }
  }

  gzdbg << "Check resting positions" << std::endl;

  // link2 of all_collide should have the highest z-coordinate (around 3)
  EXPECT_NEAR(models["all_collide"]->GetLink("link2")->GetWorldPose().pos.z,
              2.5, g_physics_tol);

  // link2 of some_collide should have a middling z-coordinate (around 2)
  EXPECT_NEAR(models["some_collide"]->GetLink("link2")->GetWorldPose().pos.z,
              1.5, g_physics_tol);

  // link2 of no_collide should have a low z-coordinate (around 1)
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->GetWorldPose().pos.z,
              0.5, g_physics_tol);

  // link2 of explicit_no_collide should have the same z-coordinate as above
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->GetWorldPose().pos.z,
     models["explicit_no_collide"]->GetLink("link2")->GetWorldPose().pos.z,
     g_physics_tol);

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
