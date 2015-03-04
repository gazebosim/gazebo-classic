/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

  EXPECT_EQ(manager->GetFilterCount(), 0u);

  // Verify that no topic is created if passing in an empty collisions
  std::vector<std::string> collisions;
  std::string emptyName = "empty";
  std::string topic  = manager->CreateFilter(emptyName, collisions);
  EXPECT_EQ(topic, "");
  EXPECT_TRUE(!manager->HasFilter(emptyName));
  EXPECT_EQ(manager->GetFilterCount(), 0u);

  // Verify we get a valid topic name after passing in collisions
  std::map<std::string, physics::CollisionPtr> collisionMap;
  collisionMap["test_collision"] = physics::CollisionPtr();
  std::string collisionMapName = "collision_map";
  topic  = manager->CreateFilter(collisionMapName, collisionMap);
  EXPECT_TRUE(topic.find(collisionMapName) != std::string::npos);
  EXPECT_TRUE(manager->HasFilter(collisionMapName));
  EXPECT_EQ(manager->GetFilterCount(), 1u);

  std::vector<std::string> collisionVector;
  collisionVector.push_back("test_collision2");
  std::string collisionVectorName = "collision_vector";
  topic = manager->CreateFilter(
      collisionVectorName, collisionVector);
  EXPECT_TRUE(topic.find(collisionVectorName) != std::string::npos);
  EXPECT_TRUE(manager->HasFilter(collisionVectorName));
  EXPECT_EQ(manager->GetFilterCount(), 2u);
}

/////////////////////////////////////////////////
TEST_F(ContactManagerTest, RemoveFilter)
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

  // Add one filter then remove it
  std::map<std::string, physics::CollisionPtr> collisionMap;
  collisionMap["test_collision"] = physics::CollisionPtr();
  std::string collisionMapName = "collision_map";
  std::string topic  = manager->CreateFilter(collisionMapName, collisionMap);
  EXPECT_TRUE(topic.find(collisionMapName) != std::string::npos);
  EXPECT_TRUE(manager->HasFilter(collisionMapName));
  EXPECT_EQ(manager->GetFilterCount(), 1u);
  // Verify that the filter is removed
  manager->RemoveFilter(collisionMapName);
  EXPECT_TRUE(!manager->HasFilter(collisionMapName));
  EXPECT_EQ(manager->GetFilterCount(), 0u);

  // Add more filters then remove them one by one
  std::string name = "collisions";
  unsigned int runs = 5;
  for (unsigned int i = 0; i < runs; ++i)
  {
    std::stringstream ss;
    ss << name << i;
    std::map<std::string, physics::CollisionPtr> collisions;
    collisionMap["collision"] = physics::CollisionPtr();
    ASSERT_TRUE(collisionMap["collision"] == NULL);

    manager->CreateFilter(ss.str(), collisions);
    EXPECT_TRUE(manager->HasFilter(ss.str()));
    EXPECT_EQ(manager->GetFilterCount(), i+1);
  }
  for (unsigned int i = 0; i < runs; ++i)
  {
    std::stringstream ss;
    ss << name << i;
    manager->RemoveFilter(ss.str());
    EXPECT_TRUE(!manager->HasFilter(ss.str()));
    EXPECT_EQ(manager->GetFilterCount(), runs - (i+1));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
