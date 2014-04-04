/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"
#include "gazebo/math/Box.hh"

using namespace gazebo;

class BoundingBoxTest: public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  /// \brief Test for collision bounding box computation.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void Collision(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void BoundingBoxTest::Collision(const std::string &_physicsEngine)
{
  Load("worlds/box_plane_low_friction_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link);

  physics::CollisionPtr coll = link->GetCollision("collision");
  ASSERT_TRUE(coll);

  EXPECT_EQ(coll->GetCollisionBoundingBox(),
      math::Box(math::Vector3(-0.5, -0.5, 0), math::Vector3(0.5, 0.5, 1)));

  // Move the box
  model->SetWorldPose(math::Pose(10, 15, 20, 0, 0, 0));

  EXPECT_EQ(coll->GetCollisionBoundingBox(),
      math::Box(math::Vector3(9.5, 14.5, 19.5),
                math::Vector3(10.5, 15.5, 20.5)));
}

/////////////////////////////////////////////////
TEST_P(BoundingBoxTest, Collision)
{
  std::string physicsEngine(GetParam());
  if (physicsEngine.compare("ode") != 0)
  {
    gzerr << "Skip test for " << physicsEngine
          << " per issue #1148"
          << std::endl;
    return;
  }
  Collision(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, BoundingBoxTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
