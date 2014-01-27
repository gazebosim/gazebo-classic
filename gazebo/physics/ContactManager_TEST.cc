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

#include "gazebo/physics/ContactManager.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;

class ContactManagerTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(ContactManagerTest, CreateFilter)
{
  Load("worlds/empty.world", false);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  physics::ContactManager *manager = physics->GetContactManager();
  ASSERT_TRUE(physics != NULL);

  // Verify that no topic is created if passing in an empty collisions
  std::vector<std::string> collisions;
  std::string topic  = manager->CreateFilter("empty", collisions);
  EXPECT_TRUE(topic == "");

  // Verify we get a valid topic name after passing in collisions
  std::map<std::string, physics::CollisionPtr> collisionMap;
  collisionMap["test_collision"] = physics::CollisionPtr();
  std::string collisionMapName = "collision_map";
  topic  = manager->CreateFilter(collisionMapName, collisionMap);
  EXPECT_TRUE(topic.find(collisionMapName) != std::string::npos);

  std::vector<std::string> collisionVector;
  collisionVector.push_back("test_collision2");
  std::string collisionVectorName = "collision_vector";
  topic  = manager->CreateFilter(
      collisionVectorName, collisionVector);
  EXPECT_TRUE(topic.find(collisionVectorName) != std::string::npos);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
