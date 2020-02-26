/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;
class ContactsUpdate : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  /// \brief Spawns two spheres which intersect, disables the
  /// collision engine and checks that contact points are still generated.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void TestTwoSpheres(const std::string &_physicsEngine);
};

void OnContact(ConstContactsPtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
void ContactsUpdate::TestTwoSpheres(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Load the spheres into the world.
  // Set position high enough so it doesn't intersect ground plane
  ignition::math::Vector3d pos(0, 0, 5);
  this->SpawnSphere("sphere1", pos, ignition::math::Vector3d::Zero, false);
  // Spheres spawned with SpawnSphere() have a radius of 0.5.
  // Move it up so it intersects with the other sphere
  pos.Z() += 0.4;
  this->SpawnSphere("sphere2", pos, ignition::math::Vector3d::Zero, true);

  // Get a pointer to the world, physics engine and contact manager
  physics::WorldPtr world = physics::get_world("default");

  // Set gravity to zero to make sure the spheres do not fall on the ground
  world->SetGravity(ignition::math::Vector3d());

  ASSERT_TRUE(world != nullptr);
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  physics::ContactManager * contactManager = physics->GetContactManager();
  ASSERT_TRUE(contactManager != nullptr);
  // Set contact manager to never drop contacts, even if there are
  // no subscribers.
  contactManager->SetNeverDropContacts(true);

  // Disable physics engine and do one step.
  // The contacts should be available, even though the engine is disabled.
  world->SetPhysicsEnabled(false);
  EXPECT_FALSE(world->PhysicsEnabled());

  world->Step(1);

  gzdbg << "Number of contacts: " << contactManager->GetContactCount() << "\n";
  EXPECT_GT(contactManager->GetContactCount(), 0u);

  contactManager->ResetCount();

  world->SetPhysicsEnabled(true);
  // Enable the engine and do one step.
  // The contacts should be available with the engine enabled.
  world->Step(1);

  gzdbg << "Number of contacts: " << contactManager->GetContactCount() << "\n";
  EXPECT_GT(contactManager->GetContactCount(), 0u);

  world->RemoveModel("sphere2");
  // There should be no more contacts reported after sphere2 is removed.
  world->Step(1);

  EXPECT_EQ(contactManager->GetContactCount(), 0u);
}

TEST_P(ContactsUpdate, TestTwoSpheres)
{
  TestTwoSpheres(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ContactsUpdate, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
