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
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"
#include "helper_physics_generator.hh"

#define TOL 1e-4

using namespace gazebo;
class ContactSensor : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  public: void EmptyWorld(const std::string &_physicsEngine);
  public: void StackTest(const std::string &_physicsEngine);
  public: void TorqueTest(const std::string &_physicsEngine);
};

void ContactSensor::EmptyWorld(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
}

TEST_P(ContactSensor, EmptyWorld)
{
  EmptyWorld(GetParam());
}

////////////////////////////////////////////////////////////////////////
// Test contact sensor using two configurations. Both place a sphere over
// a box collision contact sensor. It is expected that there is one point of
// contact in the negative z direction (world frame). The second contact sensor
// model differ from the first in that it is rotated by 90 degrees about the
// y axis. The test verifies that the feedback is given in the correct
// reference frame
////////////////////////////////////////////////////////////////////////
void ContactSensor::StackTest(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #865.\n";
    return;
  }

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
  ignition::math::Pose3d modelPose01(0, 0, 0.5, 0, 0, 0);

  std::string modelName02 = "contactModel02";
  std::string contactSensorName02 = "contactSensor02";
  ignition::math::Pose3d modelPose02(0, 2, 0.5, 0, M_PI/2.0, 0);

  std::string sphereName01 = "sphere01";
  ignition::math::Pose3d spherePose01(0, 0, 1.5, 0, 0, 0);

  std::string sphereName02 = "sphere02";
  ignition::math::Pose3d spherePose02(0, 2, 1.5, 0, 0, 0);

  // spawn two contact sensors
  SpawnUnitContactSensor(modelName01, contactSensorName01,
      "box", modelPose01.Pos(), modelPose01.Rot().Euler());
  SpawnUnitContactSensor(modelName02, contactSensorName02,
      "box", modelPose02.Pos(), modelPose02.Rot().Euler());

  // spawn two spheres, each sphere rests on top of one contact sensor
  SpawnSphere(sphereName01, spherePose01.Pos(), spherePose01.Rot().Euler());
  SpawnSphere(sphereName02, spherePose02.Pos(), spherePose02.Rot().Euler());

  sensors::SensorPtr sensor01 = sensors::get_sensor(contactSensorName01);
  sensors::ContactSensorPtr contactSensor01 =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor01);

  ASSERT_TRUE(contactSensor01 != NULL);

  sensors::SensorPtr sensor02 = sensors::get_sensor(contactSensorName02);
  sensors::ContactSensorPtr contactSensor02 =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor02);

  ASSERT_TRUE(contactSensor02 != NULL);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor01->IsActive());
  EXPECT_FALSE(contactSensor02->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor01->GetCollisionCount(), expectedColCount);
  EXPECT_EQ(contactSensor02->GetCollisionCount(), expectedColCount);

  // make sure the sensors are active and publishing
  contactSensor01->SetActive(true);
  contactSensor02->SetActive(true);

  EXPECT_TRUE(contactSensor01->IsActive());
  EXPECT_TRUE(contactSensor02->IsActive());

  physics::ModelPtr contactModel01 = world->GetModel(modelName01);
  physics::ModelPtr contactModel02 = world->GetModel(modelName02);
  ASSERT_TRUE(contactModel01 != NULL);
  ASSERT_TRUE(contactModel02 != NULL);

  std::vector<physics::ModelPtr> models;
  models.push_back(contactModel01);
  models.push_back(contactModel02);

  double gravityZ = -9.8;
  physics->SetGravity(ignition::math::Vector3d(0, 0, gravityZ));

  msgs::Contacts contacts01;
  msgs::Contacts contacts02;

  // let objects stablize
  world->Step(1000);

  int steps = 1000;
  while ((contacts01.contact_size() == 0 || contacts02.contact_size() == 0)
      && --steps > 0)
  {
    world->Step(1);
    contacts01 = contactSensor01->GetContacts();
    contacts02 = contactSensor02->GetContacts();
  }
  EXPECT_GT(steps, 0);

  std::vector<msgs::Contacts> contacts;
  contacts.push_back(contacts01);
  contacts.push_back(contacts02);

  ignition::math::Vector3d expectedForce;
  ignition::math::Vector3d expectedTorque;

  // double tolPercentage = 0.1;
  // double tol = 1e-2;
  // double tolX, tolY, tolZ;

  // Run the test once for each contact sensor
  for (unsigned int k = 0; k < contacts.size(); ++k)
  {
    double mass = models[k]->GetLink()->GetInertial()->GetMass();
    expectedForce = models[k]->GetLink()->GetWorldCoGPose().Rot().Inverse()
        * ignition::math::Vector3d(0, 0, (gravityZ * mass));
    expectedTorque = ignition::math::Vector3d(0, 0, 0);

    unsigned int ColInd = 0;
    physics::CollisionPtr col = models[k]->GetLink()->GetCollision(ColInd);
    ASSERT_TRUE(col != NULL);

    // calculate tolerance based on magnitude of force
    // Uncomment lines below once we are able to accurately determine the
    // expected force output, see issue #565
    // tolX = std::max(tolPercentage*expectedForce.X(), tol);
    // tolY = std::max(tolPercentage*expectedForce.Y(), tol);
    // tolZ = std::max(tolPercentage*expectedForce.Z(), tol);

    // loop through contact collision pairs
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

      ignition::math::Vector3d actualForce;
      ignition::math::Vector3d actualTorque;

      // loop through all contact points between the two collisions
      for (int j = 0; j < contacts[k].contact(i).position_size(); ++j)
      {
        // Contact between the sphere and the contact sensor occurs at z=1.0
        // Skip other contact points with the ground plane
        if (!ignition::math::equal(contacts[k].contact(i).position(j).Z(), 1.0))
          continue;

        EXPECT_NEAR(contacts[k].contact(i).position(j).X(),
            models[k]->GetLink()->GetWorldCoGPose().Pos().X(), TOL);
        EXPECT_NEAR(contacts[k].contact(i).position(j).Y(),
            models[k]->GetLink()->GetWorldCoGPose().Pos().Y(), TOL);

        EXPECT_NEAR(contacts[k].contact(i).normal(j).X(), 0, TOL);
        EXPECT_NEAR(contacts[k].contact(i).normal(j).Y(), 0, TOL);
        EXPECT_NEAR(contacts[k].contact(i).normal(j).Z(), 1, TOL);

        if (body1)
        {
          actualForce.X() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().X();
          actualForce.Y() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().Y();
          actualForce.Z() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().Z();

          actualTorque.X() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().X();
          actualTorque.Y() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().Y();
          actualTorque.Z() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().Z();
        }
        else
        {
          actualForce.X() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().X();
          actualForce.Y() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().Y();
          actualForce.Z() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().Z();

          actualTorque.X() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().X();
          actualTorque.Y() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().Y();
          actualTorque.Z() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().Z();
        }

        // Find the dominant force vector component and verify the value has
        // the correct sign.
        // Force and torque are given in the link frame, see issue #545
        int vi = (fabs(expectedForce.X()) > fabs(expectedForce.Y()))
            ? 0 : 1;
        vi = (fabs(expectedForce[vi]) > fabs(expectedForce.Z()))
            ? vi : 2;
        EXPECT_EQ((expectedForce[vi] < 0), (actualForce[vi] < 0));

        // Verify torque with a large tolerance
        double odeTorqueTol = 2;
        EXPECT_LT(fabs(actualTorque.X()), odeTorqueTol);
        EXPECT_LT(fabs(actualTorque.Y()), odeTorqueTol);
        EXPECT_LT(fabs(actualTorque.Z()), odeTorqueTol);

        // TODO: ODE and bullet produce slightly different results
        // In ODE the force and torque on an object seem to be affected by other
        // components like friction, e.g. It is found that the stationary sphere
        // exerts non-zero x and y forces on the contact sensor when only
        // negative z forces are expected.
        // The tests below pass in bullet but fail in ode, see issue #565
        // EXPECT_NEAR(expectedForce.X(), actualForce.X(), tolX);
        // EXPECT_NEAR(expectedForce.Y(), actualForce.Y(), tolY);
        // EXPECT_NEAR(expectedForce.Z(), actualForce.Z(), tolZ);
        // EXPECT_NEAR(expectedTorque.X(), actualTorque.X(), tolX);
        // EXPECT_NEAR(expectedTorque.Y(), actualTorque.Y(), tolY);
        // EXPECT_NEAR(expectedTorque.Z(), actualTorque.Z(), tolZ);
      }
    }
  }
}

TEST_P(ContactSensor, StackTest)
{
  StackTest(GetParam());
}

////////////////////////////////////////////////////////////////////////
// Test contact sensor torque feedback. Rest one x-rotated cylinder over
// a y-rotated cylinder so they form a cross shape. Position the top cylinder
// slightly to the -y direction so it begins to topple when simulation starts,
// then verify it's torque
////////////////////////////////////////////////////////////////////////
void ContactSensor::TorqueTest(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #865.\n";
    return;
  }

  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::string modelName = "contactModel";
  std::string contactSensorName = "contactSensor";
  ignition::math::Pose3d modelPose(0, -0.3, 1.5, M_PI/2.0, 0, 0);

  std::string cylinderName = "cylinder";
  ignition::math::Pose3d cylinderPose(0, 0, 0.5, 0, M_PI/2.0, 0);

  SpawnUnitContactSensor(modelName, contactSensorName,
      "cylinder", modelPose.Pos(), modelPose.Rot().Euler());

  SpawnCylinder(cylinderName, cylinderPose.Pos(), cylinderPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(contactSensorName);
  sensors::ContactSensorPtr contactSensor =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

  ASSERT_TRUE(contactSensor != NULL);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor->GetCollisionCount(), expectedColCount);

  contactSensor->SetActive(true);

  EXPECT_TRUE(contactSensor->IsActive());

  physics::ModelPtr contactModel = world->GetModel(modelName);
  ASSERT_TRUE(contactModel != NULL);

  double gravityZ = -9.8;
  physics->SetGravity(ignition::math::Vector3d(0, 0, gravityZ));

  msgs::Contacts contacts;

  physics->SetContactMaxCorrectingVel(0);
  physics->SetParam("iters", 100);

  world->Step(1);

  // run simulation until contacts occur
  int steps = 2000;
  while (contacts.contact_size() == 0 && --steps > 0)
  {
    world->Step(1);
    contacts = contactSensor->GetContacts();
  }

  EXPECT_GT(steps, 0);

  contacts = contactSensor->GetContacts();

  unsigned int ColInd = 0;
  physics::CollisionPtr col = contactModel->GetLink()->GetCollision(ColInd);
  ASSERT_TRUE(col != NULL);

  // double tol = 2e-1;
  // loop through contact collision pairs
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    bool body1 = true;
    if (contacts.contact(i).collision1() == col->GetScopedName())
      body1 = true;
    else if (contacts.contact(i).collision2() == col->GetScopedName())
      body1 = false;
    else
    {
      FAIL();
    }
    ignition::math::Vector3d actualTorque;

    // loop through all contact points between the two collisions
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      if (contacts.contact(i).position(j).Z() < 0.5)
        continue;

      if (body1)
      {
        actualTorque.X() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().X();
        actualTorque.Y() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().Y();
        actualTorque.Z() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().Z();
      }
      else
      {
        actualTorque.X() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().X();
        actualTorque.Y() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().Y();
        actualTorque.Z() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().Z();
      }

      // dart doesn't pass this portion of the test (#910)
      if (_physicsEngine != "dart")
      {
        // contact sensor should have positive x torque and relatively large
        // compared to y and z
        EXPECT_GT(actualTorque.X(), 0);
        EXPECT_GT(actualTorque.X(), fabs(actualTorque.Y()));
        EXPECT_GT(actualTorque.X(), fabs(actualTorque.Z()));
        // EXPECT_LT(fabs(actualTorque.Y()), tol);
        // EXPECT_LT(fabs(actualTorque.Z()), tol);
      }
    }
  }
}

TEST_P(ContactSensor, TorqueTest)
{
  TorqueTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ContactSensor, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
