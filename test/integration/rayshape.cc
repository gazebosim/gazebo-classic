/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class RayShapeTest : public ServerFixture,
                     public testing::WithParamInterface<const char*>
{
  public: void Standalone(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void RayShapeTest::Standalone(const std::string &_physicsEngine)
{
  // Load the shapes world
  Load("worlds/shapes.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        world->GetPhysicsEngine()->CreateShape("ray",
          gazebo::physics::CollisionPtr()));
  ASSERT_TRUE(ray != NULL);

  double dist;
  std::string entity;

  ray->SetPoints(ignition::math::Vector3d(-1, 0, 0.5),
                 ignition::math::Vector3d(10, 0, 0.5));
  ray->GetIntersection(dist, entity);

  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "box::link::collision");

  ray->SetPoints(ignition::math::Vector3d(-1, 1.5, 0.5),
                 ignition::math::Vector3d(10, 1.5, 0.5));
  ray->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "sphere::link::collision");

  ray->SetPoints(ignition::math::Vector3d(-1, -1.5, 0.5),
                 ignition::math::Vector3d(10, -1.5, 0.5));
  ray->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "cylinder::link::collision");

  ray->SetPoints(ignition::math::Vector3d(-1, -10.5, 0.5),
                 ignition::math::Vector3d(10, -10.5, 0.5));
  ray->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 1000, 1e-4);
  EXPECT_TRUE(entity.empty());
}

/////////////////////////////////////////////////
TEST_P(RayShapeTest, Standalone)
{
  Standalone(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, RayShapeTest,
    ::testing::Values("ode"));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
