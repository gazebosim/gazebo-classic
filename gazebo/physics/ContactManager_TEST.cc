/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gazebo/test/ServerFixture.hh"

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
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);

  physics::ContactManager *manager = physics->GetContactManager();
  ASSERT_TRUE(physics != nullptr);

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
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);

  physics::ContactManager *manager = physics->GetContactManager();
  ASSERT_TRUE(physics != nullptr);

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
    ASSERT_TRUE(collisionMap["collision"] == nullptr);

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
  EXPECT_EQ(manager->GetFilterCount(), 0u);

  // Add and remove filter with :: in name
  collisionMapName = "link::collision";
  collisionMap[collisionMapName] = physics::CollisionPtr();
  topic  = manager->CreateFilter(collisionMapName, collisionMap);
  EXPECT_TRUE(topic.find("link/collision") != std::string::npos);
  EXPECT_TRUE(manager->HasFilter(collisionMapName));
  EXPECT_EQ(manager->GetFilterCount(), 1u);
  // Verify that the filter is removed
  manager->RemoveFilter(collisionMapName);
  EXPECT_FALSE(manager->HasFilter(collisionMapName));
  EXPECT_EQ(manager->GetFilterCount(), 0u);
}

/////////////////////////////////////////////////
TEST_F(ContactManagerTest, NeverDropContacts)
{
  // world needs to be paused in order to use World::Step()
  // function correctly (second parameter true)
  Load("test/worlds/box.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);

  physics::ContactManager *manager = physics->GetContactManager();
  ASSERT_TRUE(manager != nullptr);

  // advance the world. Contacts should happen between
  // box and ground.
  world->Step(1);

  unsigned int numContacts = manager->GetContactCount();

  // we have not enforced contacts computation (yet), so
  // we should not have access to any contacts information
  // without the enforcement.
  ASSERT_EQ(numContacts, 0u);

  manager->SetNeverDropContacts(true);
  ASSERT_TRUE(manager->NeverDropContacts());

  // advance the world again, this time the contacts
  // information should become available.
  world->Step(1);

  numContacts = manager->GetContactCount();
  gzdbg << "Number of contacts: " <<numContacts << std::endl;
  ASSERT_GT(numContacts, 0u);

  // make sure there are no subscribers connected which may have
  // caused the contacts to become true
  const std::vector<physics::Contact *>& contacts = manager->GetContacts();
  for (std::vector<physics::Contact *>::const_iterator it = contacts.begin();
       it != contacts.end(); ++it)
  {
    physics::Contact* contact = *it;
    physics::Collision* coll1 = contact->collision1;
    physics::Collision* coll2 = contact->collision2;
    ASSERT_TRUE(coll1 != nullptr);
    ASSERT_TRUE(coll2 != nullptr);
    // we have no subscribers connected and still have gotten the contacts
    // information, which means that enforcing contacts computation has worked.
    ASSERT_FALSE(manager->SubscribersConnected(coll1, coll2));
    ASSERT_FALSE(manager->SubscribersConnected(coll2, coll1));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
