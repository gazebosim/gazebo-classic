/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include "test/ServerFixture.hh"

#include "gazebo/physics/RayShape.hh"

using namespace gazebo;
using namespace physics;

class RayShape_TEST : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  public: RayShape_TEST();
  public: void GetIntersectionEmptySpace(const std::string &_physicsEngine);
  public: void GetIntersectionObstacle(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
/// \brief Constructor
RayShape_TEST::RayShape_TEST()
{
}

/////////////////////////////////////////////////
/// \brief Test GetIntersection of a testRay without any obstacles
void RayShape_TEST::GetIntersectionEmptySpace(const std::string &_physicsEngine)
{
  std::string entityName;
  double dist;

  Load("worlds/empty.world", false, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Create a RayShape
  physics::RayShapePtr testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  math::Vector3 end(0.0, 0.0, 0.5);
  math::Vector3 start(10.0, 0.0, 0.5);

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(world->GetPhysicsEngine()->
      GetPhysicsUpdateMutex()));

    // Looking for obstacles between start and end points
  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);

  // DEBUG output
  std::cout << "Distance: [" << dist << "]\n";
  std::cout << "Entity: [" << entityName << "]\n";

  EXPECT_TRUE(dist == 0.0 && entityName == "");
}

/////////////////////////////////////////////////
/// \brief Test GetIntersection of a testRay with an obstacle
void RayShape_TEST::GetIntersectionObstacle(const std::string &_physicsEngine)
{
  std::string entityName;
  double dist;

  Load("worlds/shapes.world", false, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Create a RayShape
  physics::RayShapePtr testRay = boost::dynamic_pointer_cast<RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  math::Vector3 end(-10.0, 0.0, 0.5);
  math::Vector3 start(10.0, 0.0, 0.5);

  // Acquire the mutex for avoiding race condition with the physics engine
  boost::recursive_mutex::scoped_lock lock(*(world->GetPhysicsEngine()->
      GetPhysicsUpdateMutex()));

    // Looking for obstacles between start and end points
  testRay->SetPoints(start, end);
  testRay->GetIntersection(dist, entityName);

  // DEBUG output
  std::cout << "Distance: [" << dist << "]\n";
  std::cout << "Entity: [" << entityName << "]\n";

  EXPECT_TRUE(dist == 9.5 && entityName != "");
}

/////////////////////////////////////////////////
TEST_P(RayShape_TEST, EmptySpace)
{
  GetIntersectionEmptySpace(GetParam());
}

/////////////////////////////////////////////////
TEST_P(RayShape_TEST, Obstacle)
{
  GetIntersectionObstacle(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(TestRayShapeODE, RayShape_TEST,
    ::testing::Values("ode"));

/////////////////////////////////////////////////
#ifdef HAVE_BULLET
INSTANTIATE_TEST_CASE_P(TestRayShapeBullet, RayShape_TEST,
    ::testing::Values("bullet"));
#endif  // HAVE_BULLET

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
