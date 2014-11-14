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
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "helper_physics_generator.hh"

const double g_physics_tol = 1e-2;

using namespace gazebo;

class SurfaceTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void CollideWithoutContact(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// CollideWithoutContact:
// Load the collide_without_contact test world. It drops two boxes onto
// a larger static box with a contact sensor. One of the boxes should be
// detected by the contact sensor but not experience contact forces.
////////////////////////////////////////////////////////////////////////
void SurfaceTest::CollideWithoutContact(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/collide_without_contact.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // get pointers to the falling boxes.
  physics::ModelPtr contactBox, collideBox;
  contactBox = world->GetModel("contact_box");
  collideBox = world->GetModel("collide_box");
  ASSERT_TRUE(contactBox != NULL);
  ASSERT_TRUE(collideBox != NULL);

  // get the contact sensor
  sensors::SensorPtr sensor = sensors::get_sensor("box_contact");
  sensors::ContactSensorPtr contactSensor =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
  ASSERT_TRUE(contactSensor != NULL);

  // Step forward 0.2 s
  double stepTime = 0.2;
  unsigned int steps = floor(stepTime / dt);
  world->Step(steps);

  // Expect boxes to be falling
  double fallVelocity = g.z * stepTime;
  EXPECT_LT(contactBox->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));
  EXPECT_LT(collideBox->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));

  // Step forward another 0.2 s
  world->Step(steps);
  fallVelocity = g.z * 2*stepTime;
  // Expect contactBox to be resting on contact sensor box
  EXPECT_NEAR(contactBox->GetWorldLinearVel().z, 0.0, g_physics_tol);
  // Expect collideBox to still be falling
  EXPECT_LT(collideBox->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));

  {
    // Step forward until we get a contacts message from the contact sensor
    msgs::Contacts contacts;
    while (contacts.contact_size() == 0 && --steps > 0)
    {
      world->Step(1);
      contacts = contactSensor->GetContacts();
    }

    // Verify that both objects are recognized by contact sensor
    int i;
    msgs::Contact contact;
    bool collideBoxHit = false;
    bool contactBoxHit = false;
    for (i = 0; i < contacts.contact_size(); ++i)
    {
      contact = contacts.contact(i);
      if (contact.collision1() == "contact_box::link::collision" ||
          contact.collision2() == "contact_box::link::collision")
      {
        contactBoxHit = true;
      }
      if (contact.collision1() == "collide_box::link::collision" ||
          contact.collision2() == "collide_box::link::collision")
      {
        collideBoxHit = true;
      }
    }
    EXPECT_TRUE(contactBoxHit);
    EXPECT_TRUE(collideBoxHit);
  }

  // Step forward another 0.4 s
  // The collideBox should have fallen through the ground and not
  // be in contact with the sensor
  world->Step(steps*2);
  fallVelocity = g.z * 4*stepTime;
  // Expect contactBox to still be resting on contact sensor box
  EXPECT_NEAR(contactBox->GetWorldLinearVel().z, 0.0, g_physics_tol);
  // Expect collideBox to still be falling
  EXPECT_LT(collideBox->GetWorldLinearVel().z, fallVelocity*(1-g_physics_tol));

  {
    // Step forward until we get a contacts message from the contact sensor
    msgs::Contacts contacts;
    while (contacts.contact_size() == 0 && --steps > 0)
    {
      world->Step(1);
      contacts = contactSensor->GetContacts();
    }

    // Verify that only contactBox is recognized by contact sensor
    int i;
    msgs::Contact contact;
    bool collideBoxHit = false;
    bool contactBoxHit = false;
    for (i = 0; i < contacts.contact_size(); ++i)
    {
      contact = contacts.contact(i);
      if (contact.collision1() == "contact_box::link::collision" ||
          contact.collision2() == "contact_box::link::collision")
      {
        contactBoxHit = true;
      }
      if (contact.collision1() == "collide_box::link::collision" ||
          contact.collision2() == "collide_box::link::collision")
      {
        collideBoxHit = true;
      }
    }
    EXPECT_TRUE(contactBoxHit);
    EXPECT_FALSE(collideBoxHit);
  }
}

TEST_P(SurfaceTest, CollideWithoutContact)
{
  CollideWithoutContact(GetParam());
}

// This test doesn't yet work in bullet, so we'll declare it only for ode.
// Issue #1038
// INSTANTIATE_TEST_CASE_P(PhysicsEngines, SurfaceTest, PHYSICS_ENGINE_VALUES);
INSTANTIATE_TEST_CASE_P(TestODE, SurfaceTest, ::testing::Values("ode"));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
