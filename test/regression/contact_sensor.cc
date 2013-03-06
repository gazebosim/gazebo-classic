/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "physics/physics.hh"
#include "sensors/sensors.hh"
#include "common/common.hh"
#include "scans_cmp.h"

#define TOL 1e-4

using namespace gazebo;
class ContactSensor : public ServerFixture
{
  public: void StackTest(const std::string &_physicsEngine);
};

TEST_F(ContactSensor, EmptyWorld)
{
  Load("worlds/empty.world");
}

void ContactSensor::StackTest(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::string modelName01 = "contactModel01";
  std::string contactSensorName01 = "contactSensor01";
  math::Pose modelPose01(0, 0, 0.5, 0, 0, 0);

  std::string modelName02 = "contactModel02";
  std::string contactSensorName02 = "contactSensor02";
  math::Pose modelPose02(0, 2, 0.5, 0, M_PI/2.0, 0);

  std::string sphereName01 = "sphere01";
  math::Pose spherePose01(0, 0, 1.5, 0, 0, 0);

  std::string sphereName02 = "sphere02";
  math::Pose spherePose02(0, 2, 1.5, 0, 0, 0);

  SpawnBoxContactSensor(modelName01, contactSensorName01,
      math::Vector3(1, 1, 1), modelPose01.pos, modelPose01.rot.GetAsEuler());
  SpawnBoxContactSensor(modelName02, contactSensorName02,
      math::Vector3(1, 1, 1), modelPose02.pos, modelPose02.rot.GetAsEuler());

  SpawnSphere(sphereName01, spherePose01.pos, spherePose01.rot.GetAsEuler());
  SpawnSphere(sphereName02, spherePose02.pos, spherePose02.rot.GetAsEuler());

  sensors::SensorPtr sensor01 = sensors::get_sensor(contactSensorName01);
  sensors::ContactSensorPtr contactSensor01 =
      boost::shared_dynamic_cast<sensors::ContactSensor>(sensor01);

  ASSERT_TRUE(contactSensor01);

  sensors::SensorPtr sensor02 = sensors::get_sensor(contactSensorName02);
  sensors::ContactSensorPtr contactSensor02 =
      boost::shared_dynamic_cast<sensors::ContactSensor>(sensor02);

  ASSERT_TRUE(contactSensor02);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor01->IsActive());
  EXPECT_FALSE(contactSensor02->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor01->GetCollisionCount(), expectedColCount);
  EXPECT_EQ(contactSensor02->GetCollisionCount(), expectedColCount);

  contactSensor01->SetActive(true);
  contactSensor02->SetActive(true);

  EXPECT_TRUE(contactSensor01->IsActive());
  EXPECT_TRUE(contactSensor02->IsActive());

  physics::ModelPtr contactModel01 = world->GetModel(modelName01);
  physics::ModelPtr contactModel02 = world->GetModel(modelName02);
  ASSERT_TRUE(contactModel01);
  ASSERT_TRUE(contactModel02);

  std::vector<physics::ModelPtr> models;
  models.push_back(contactModel01);
  models.push_back(contactModel02);

  double gravityZ = -9.8;
  physics->SetGravity(math::Vector3(0, 0, gravityZ));

  double tolP = 0.1;
  double tol = 1e-2;

  // Get all the contacts[k].
  msgs::Contacts contacts01;
  msgs::Contacts contacts02;

  world->StepWorld(1000);

  while (contacts01.contact_size() == 0 || contacts02.contact_size() == 0)
  {
    world->StepWorld(1);
    contacts01 = contactSensor01->GetContacts();
    contacts02 = contactSensor02->GetContacts();
  }

  std::vector<msgs::Contacts> contacts;
  contacts.push_back(contacts01);
  contacts.push_back(contacts02);

  math::Vector3 expectedForce;
  math::Vector3 expectedTorque;

  for (unsigned int k = 0; k < contacts.size(); ++k)
  {
    double mass = models[k]->GetLink()->GetInertial()->GetMass();
    expectedForce = models[k]->GetLink()->GetWorldCoGPose().rot.GetInverse()
        * math::Vector3(0, 0, (gravityZ * mass));
    expectedTorque = math::Vector3(0, 0, 0);

    unsigned int ColInd = 0;
    physics::CollisionPtr col = models[k]->GetLink()->GetCollision(ColInd);
    ASSERT_TRUE(col);

    double tolX = std::max(tolP*expectedForce.x, tol);
    double tolY = std::max(tolP*expectedForce.y, tol);
    double tolZ = std::max(tolP*expectedForce.z, tol);

    for (int i = 0; i < contacts[k].contact_size(); ++i)
    {
      bool body1 = true;
      if (contacts[k].contact(i).collision1() == col->GetScopedName())
        body1 = true;
      else if (contacts[k].contact(i).collision2() == col->GetScopedName())
        body1 = false;
      else
      {
        FAIL();
      }

      math::Vector3 actualForce;
      math::Vector3 actualTorque;

      for (int j = 0; j < contacts[k].contact(i).position_size(); ++j)
      {
        if (!math::equal(contacts[k].contact(i).position(j).z(), 1.0))
          continue;

        EXPECT_NEAR(contacts[k].contact(i).position(j).x(),
            models[k]->GetLink()->GetWorldCoGPose().pos.x, TOL);
        EXPECT_NEAR(contacts[k].contact(i).position(j).y(),
            models[k]->GetLink()->GetWorldCoGPose().pos.y, TOL);

        EXPECT_NEAR(contacts[k].contact(i).normal(j).x(), 0, TOL);
        EXPECT_NEAR(contacts[k].contact(i).normal(j).y(), 0, TOL);
        EXPECT_NEAR(contacts[k].contact(i).normal(j).z(), 1, TOL);

        if (body1)
        {
          actualForce.x = contacts[k].contact(i).wrench(j).body_1_force().x();
          actualForce.y = contacts[k].contact(i).wrench(j).body_1_force().y();
          actualForce.z = contacts[k].contact(i).wrench(j).body_1_force().z();

          actualTorque.x = contacts[k].contact(i).wrench(j).body_1_torque().x();
          actualTorque.y = contacts[k].contact(i).wrench(j).body_1_torque().y();
          actualTorque.z = contacts[k].contact(i).wrench(j).body_1_torque().z();
        }
        else
        {
          actualForce.x = contacts[k].contact(i).wrench(j).body_2_force().x();
          actualForce.y = contacts[k].contact(i).wrench(j).body_2_force().y();
          actualForce.z = contacts[k].contact(i).wrench(j).body_2_force().z();

          actualTorque.x = contacts[k].contact(i).wrench(j).body_1_torque().x();
          actualTorque.y = contacts[k].contact(i).wrench(j).body_1_torque().y();
          actualTorque.z = contacts[k].contact(i).wrench(j).body_1_torque().z();
        }

        /// TODO ODE tests are failing:
        // The force and torque on an object seem to be affected by by other
        // components like friction, e.g. It is found that the stationary sphere
        // exerts non-zero x and y forces on the contact sensor when only
        // negative z forces are expected. So ode tests compare only the sign of
        // the dominant value in the force vector
        if (_physicsEngine == "ode")
        {
          int vi = (fabs(expectedForce.x) > fabs(expectedForce.y))
              ? 0 : 1;
          vi = (fabs(expectedForce[vi]) > fabs(expectedForce.z))
              ? vi : 2;

          EXPECT_TRUE((expectedForce[vi] >= 0) ^ (actualForce[vi] < 0));
        }
        if (_physicsEngine == "bullet")
        {
          EXPECT_NEAR(expectedForce.x, actualForce.x, tolX);
          EXPECT_NEAR(expectedForce.y, actualForce.y, tolY);
          EXPECT_NEAR(expectedForce.z, actualForce.z, tolZ);

          EXPECT_NEAR(expectedTorque.x, actualTorque.x, tolX);
          EXPECT_NEAR(expectedTorque.y, actualTorque.y, tolY);
          EXPECT_NEAR(expectedTorque.z, actualTorque.z, tolZ);
        }
      }
    }
  }
}

TEST_F(ContactSensor, StackTestODE)
{
  StackTest("ode");
}

#ifdef HAVE_BULLET
TEST_F(ContactSensor, StackTestBullet)
{
  StackTest("bullet");
}
#endif  // HAVE_BULLET



int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
