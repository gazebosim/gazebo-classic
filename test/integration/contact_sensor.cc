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
  /// \brief Test multiple contact sensors on a single link.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MultipleSensors(const std::string &_physicsEngine);
  public: void StackTest(const std::string &_physicsEngine);
  public: void TorqueTest(const std::string &_physicsEngine);

  /// \brief Callback for sensor subscribers in MultipleSensors test.
  private: void Callback(const ConstContactsPtr &_msg);
};

unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void ContactSensor::Callback(const ConstContactsPtr &/*_msg*/)
{
  g_messageCount++;
}

////////////////////////////////////////////////////////////////////////
// Test having multiple contact sensors under a single link.
// https://bitbucket.org/osrf/gazebo/issue/960
////////////////////////////////////////////////////////////////////////
void ContactSensor::MultipleSensors(const std::string &_physicsEngine)
{
  Load("worlds/contact_sensors_multiple.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);

  const std::string contactSensorName1("box_contact");
  const std::string contactSensorName2("box_contact2");

  {
    sensors::SensorPtr sensor1 = sensors::get_sensor(contactSensorName1);
    sensors::ContactSensorPtr contactSensor1 =
        boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor1);
    ASSERT_TRUE(contactSensor1 != NULL);
  }

  {
    sensors::SensorPtr sensor2 = sensors::get_sensor(contactSensorName2);
    sensors::ContactSensorPtr contactSensor2 =
        boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor2);
    ASSERT_TRUE(contactSensor2 != NULL);
  }

  // There should be 5 topics advertising Contacts messages
  std::list<std::string> topicsExpected;
  std::string prefix = "/gazebo/default/";
  topicsExpected.push_back(prefix+"physics/contacts");
  topicsExpected.push_back(prefix+"sensor_box/link/box_contact/contacts");
  topicsExpected.push_back(prefix+"sensor_box/link/box_contact");
  topicsExpected.push_back(prefix+"sensor_box/link/box_contact2/contacts");
  topicsExpected.push_back(prefix+"sensor_box/link/box_contact2");
  std::list<std::string> topics =
    transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), topicsExpected.size());
  EXPECT_EQ(topics, topicsExpected);

  // We should expect them all to publish.
  for (std::list<std::string>::iterator iter = topics.begin();
                                       iter != topics.end(); ++iter)
  {
    gzdbg << "Listening to " << *iter << std::endl;
    g_messageCount = 0;
    transport::SubscriberPtr sub = this->node->Subscribe(*iter,
      &ContactSensor::Callback, this);

    const int steps = 50;
    world->Step(steps);
    common::Time::MSleep(steps);
    EXPECT_GT(g_messageCount, steps / 2);

    sub->Unsubscribe();
    common::Time::MSleep(steps);
  }
}

TEST_P(ContactSensor, MultipleSensors)
{
  MultipleSensors(GetParam());
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

  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for DART, see issue #1173.\n";
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
  math::Pose modelPose01(0, 0, 0.5, 0, 0, 0);

  std::string modelName02 = "contactModel02";
  std::string contactSensorName02 = "contactSensor02";
  math::Pose modelPose02(0, 2, 0.5, 0, M_PI/2.0, 0);

  std::string sphereName01 = "sphere01";
  math::Pose spherePose01(0, 0, 1.5, 0, 0, 0);

  std::string sphereName02 = "sphere02";
  math::Pose spherePose02(0, 2, 1.5, 0, 0, 0);

  // spawn two contact sensors
  SpawnUnitContactSensor(modelName01, contactSensorName01,
      "box", modelPose01.pos, modelPose01.rot.GetAsEuler());
  SpawnUnitContactSensor(modelName02, contactSensorName02,
      "box", modelPose02.pos, modelPose02.rot.GetAsEuler());

  // spawn two spheres, each sphere rests on top of one contact sensor
  SpawnSphere(sphereName01, spherePose01.pos, spherePose01.rot.GetAsEuler());
  SpawnSphere(sphereName02, spherePose02.pos, spherePose02.rot.GetAsEuler());

  sensors::SensorPtr sensor01 = sensors::get_sensor(contactSensorName01);
  sensors::ContactSensorPtr contactSensor01 =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor01);

  ASSERT_TRUE(contactSensor01);

  sensors::SensorPtr sensor02 = sensors::get_sensor(contactSensorName02);
  sensors::ContactSensorPtr contactSensor02 =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor02);

  ASSERT_TRUE(contactSensor02);

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
  ASSERT_TRUE(contactModel01);
  ASSERT_TRUE(contactModel02);

  std::vector<physics::ModelPtr> models;
  models.push_back(contactModel01);
  models.push_back(contactModel02);

  double gravityZ = -9.8;
  physics->SetGravity(math::Vector3(0, 0, gravityZ));

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

  math::Vector3 expectedForce;
  math::Vector3 expectedTorque;

  // double tolPercentage = 0.1;
  // double tol = 1e-2;
  // double tolX, tolY, tolZ;

  // Run the test once for each contact sensor
  for (unsigned int k = 0; k < contacts.size(); ++k)
  {
    double mass = models[k]->GetLink()->GetInertial()->GetMass();
    expectedForce = models[k]->GetLink()->GetWorldCoGPose().rot.GetInverse()
        * math::Vector3(0, 0, (gravityZ * mass));
    expectedTorque = math::Vector3(0, 0, 0);

    unsigned int ColInd = 0;
    physics::CollisionPtr col = models[k]->GetLink()->GetCollision(ColInd);
    ASSERT_TRUE(col);

    // calculate tolerance based on magnitude of force
    // Uncomment lines below once we are able to accurately determine the
    // expected force output, see issue #565
    // tolX = std::max(tolPercentage*expectedForce.x, tol);
    // tolY = std::max(tolPercentage*expectedForce.y, tol);
    // tolZ = std::max(tolPercentage*expectedForce.z, tol);

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

      math::Vector3 actualForce;
      math::Vector3 actualTorque;

      // loop through all contact points between the two collisions
      for (int j = 0; j < contacts[k].contact(i).position_size(); ++j)
      {
        // Contact between the sphere and the contact sensor occurs at z=1.0
        // Skip other contact points with the ground plane
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
          actualForce.x =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().x();
          actualForce.y =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().y();
          actualForce.z =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().z();

          actualTorque.x =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().x();
          actualTorque.y =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().y();
          actualTorque.z =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().z();
        }
        else
        {
          actualForce.x =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().x();
          actualForce.y =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().y();
          actualForce.z =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().z();

          actualTorque.x =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().x();
          actualTorque.y =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().y();
          actualTorque.z =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().z();
        }

        // Find the dominant force vector component and verify the value has
        // the correct sign.
        // Force and torque are given in the link frame, see issue #545
        int vi = (fabs(expectedForce.x) > fabs(expectedForce.y))
            ? 0 : 1;
        vi = (fabs(expectedForce[vi]) > fabs(expectedForce.z))
            ? vi : 2;
        EXPECT_EQ((expectedForce[vi] < 0), (actualForce[vi] < 0));

        // Verify torque with a large tolerance
        double odeTorqueTol = 2;
        EXPECT_LT(fabs(actualTorque.x), odeTorqueTol);
        EXPECT_LT(fabs(actualTorque.y), odeTorqueTol);
        EXPECT_LT(fabs(actualTorque.z), odeTorqueTol);

        // TODO: ODE and bullet produce slightly different results
        // In ODE the force and torque on an object seem to be affected by other
        // components like friction, e.g. It is found that the stationary sphere
        // exerts non-zero x and y forces on the contact sensor when only
        // negative z forces are expected.
        // The tests below pass in bullet but fail in ode, see issue #565
        // EXPECT_NEAR(expectedForce.x, actualForce.x, tolX);
        // EXPECT_NEAR(expectedForce.y, actualForce.y, tolY);
        // EXPECT_NEAR(expectedForce.z, actualForce.z, tolZ);
        // EXPECT_NEAR(expectedTorque.x, actualTorque.x, tolX);
        // EXPECT_NEAR(expectedTorque.y, actualTorque.y, tolY);
        // EXPECT_NEAR(expectedTorque.z, actualTorque.z, tolZ);
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
  math::Pose modelPose(0, -0.3, 1.5, M_PI/2.0, 0, 0);

  std::string cylinderName = "cylinder";
  math::Pose cylinderPose(0, 0, 0.5, 0, M_PI/2.0, 0);

  SpawnUnitContactSensor(modelName, contactSensorName,
      "cylinder", modelPose.pos, modelPose.rot.GetAsEuler());

  SpawnCylinder(cylinderName, cylinderPose.pos, cylinderPose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(contactSensorName);
  sensors::ContactSensorPtr contactSensor =
      boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

  ASSERT_TRUE(contactSensor);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor->GetCollisionCount(), expectedColCount);

  contactSensor->SetActive(true);

  EXPECT_TRUE(contactSensor->IsActive());

  physics::ModelPtr contactModel = world->GetModel(modelName);
  ASSERT_TRUE(contactModel);

  double gravityZ = -9.8;
  physics->SetGravity(math::Vector3(0, 0, gravityZ));

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
  ASSERT_TRUE(col);

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
    math::Vector3 actualTorque;

    // loop through all contact points between the two collisions
    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      if (contacts.contact(i).position(j).z() < 0.5)
        continue;

      if (body1)
      {
        actualTorque.x =
          contacts.contact(i).wrench(j).body_1_wrench().torque().x();
        actualTorque.y =
          contacts.contact(i).wrench(j).body_1_wrench().torque().y();
        actualTorque.z =
          contacts.contact(i).wrench(j).body_1_wrench().torque().z();
      }
      else
      {
        actualTorque.x =
          contacts.contact(i).wrench(j).body_2_wrench().torque().x();
        actualTorque.y =
          contacts.contact(i).wrench(j).body_2_wrench().torque().y();
        actualTorque.z =
          contacts.contact(i).wrench(j).body_2_wrench().torque().z();
      }

      // dart doesn't pass this portion of the test (#910)
      if (_physicsEngine != "dart")
      {
        // contact sensor should have positive x torque and relatively large
        // compared to y and z
        EXPECT_GT(actualTorque.x, 0);
        EXPECT_GT(actualTorque.x, fabs(actualTorque.y));
        EXPECT_GT(actualTorque.x, fabs(actualTorque.z));
        // EXPECT_LT(fabs(actualTorque.y), tol);
        // EXPECT_LT(fabs(actualTorque.z), tol);
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
