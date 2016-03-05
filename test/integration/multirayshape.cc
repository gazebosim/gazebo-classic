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

class MultirayShapeTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
  public: void Standalone(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void MultirayShapeTest::Standalone(const std::string &_physicsEngine)
{
  // Load the shapes world
  Load("worlds/shapes.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  gazebo::physics::MultiRayShapePtr rays =
    boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
        world->GetPhysicsEngine()->CreateShape("multiray",
          gazebo::physics::CollisionPtr()));
  ASSERT_TRUE(rays != NULL);

  rays->AddRay(ignition::math::Vector3d(-1, 0, 0.5),
               ignition::math::Vector3d(10, 0, 0.5));

  rays->AddRay(ignition::math::Vector3d(-1, 1.5, 0.5),
               ignition::math::Vector3d(10, 1.5, 0.5));

  rays->AddRay(ignition::math::Vector3d(-1, -1.5, 0.5),
               ignition::math::Vector3d(10, -1.5, 0.5));

  rays->AddRay(ignition::math::Vector3d(-1, -10.5, 0.5),
               ignition::math::Vector3d(10, -10.5, 0.5));

  EXPECT_EQ(rays->RayCount(), 4u);

  rays->Update();

  double dist;
  std::string entity;

  rays->Ray(0)->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "box::link::collision");

  rays->Ray(1)->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "sphere::link::collision");

  rays->Ray(2)->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 0.5, 1e-4);
  EXPECT_EQ(entity, "cylinder::link::collision");

  rays->Ray(3)->GetIntersection(dist, entity);
  EXPECT_NEAR(dist, 1000, 1e-4);
  EXPECT_TRUE(entity.empty());
}

/////////////////////////////////////////////////
TEST_P(MultirayShapeTest, Standalone)
{
  Standalone(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, MultirayShapeTest,
    ::testing::Values("ode"));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
