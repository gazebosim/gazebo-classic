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

#include <cmath>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL 1e-4

using namespace gazebo;
class ContactSensor : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  /// \brief Test removing a model that has a contact sensor
  public: void ModelRemoval(const std::string &_physicsEngine);

  /// \brief Test moving a model while in contact.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MoveTool(const std::string &_physicsEngine);

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
void ContactSensor::ModelRemoval(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // check initial topics count
  auto topics = transport::getAdvertisedTopics();
  int topicsCount = 0;
  for (auto iter : topics)
  {
    for (auto str : iter.second)
    {
      topicsCount++;
    }
  }
  EXPECT_GT(topicsCount, 0);

  // spanw the model
  std::string modelName = "contactModel";
  std::string contactSensorName = "contactSensor";
  ignition::math::Pose3d modelPose(0, -0.3, 1.5, M_PI/2.0, 0, 0);

  SpawnUnitContactSensor(modelName, contactSensorName,
      "cylinder", modelPose.Pos(), modelPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(contactSensorName);
  sensors::ContactSensorPtr contactSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

  ASSERT_TRUE(contactSensor != nullptr);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor->GetCollisionCount(), expectedColCount);

  contactSensor->SetActive(true);

  EXPECT_TRUE(contactSensor->IsActive());

  physics::ModelPtr contactModel = world->ModelByName(modelName);
  ASSERT_TRUE(contactModel != nullptr);

  // check new topic are published
  // there should be more than 1 new topic:
  //   1 new factory topic and 2 new contact sensor topics
  int wait = 0;
  int maxWait = 20;
  int topicsCountModel = 0;
  int topicsCountModelName = 0;
  while (topicsCountModel <= topicsCount+2 && wait < maxWait)
  {
    common::Time::MSleep(100);
    topicsCountModel = 0;
    auto modelTopics = transport::getAdvertisedTopics();
    for (auto iter : modelTopics)
    {
      for (auto str : iter.second)
      {
        topicsCountModel++;
        if (str.find(modelName) != std::string::npos)
          topicsCountModelName++;
      }
    }
    wait++;
  }
  EXPECT_GT(topicsCountModel, topicsCount+1);
  EXPECT_GT(topicsCountModelName, 0);

  // remove the model
  world->RemoveModel(contactModel);

  contactModel = world->ModelByName(modelName);
  EXPECT_TRUE(contactModel == nullptr);

  int sleep = 0;
  int maxSleep  = 20;
  while (sensors::get_sensor(contactSensorName) && sleep < maxSleep)
  {
    common::Time::MSleep(30);
    sleep++;
  }
  EXPECT_TRUE(sensors::get_sensor(contactSensorName) == nullptr);

  // wait for topics cleanup
  // verify there are no more contact sensor topics and the number of topics
  // are back to the initial condition + 1 new factory topic.
  auto topicsAfter = transport::getAdvertisedTopics();
  int j = 0;
  for (j = 0; j < 5 && topicsAfter.size() > (topics.size() + 1); ++j)
  {
    common::Time::MSleep(1000);
    topicsAfter = transport::getAdvertisedTopics();
  }
  EXPECT_LT(j, 5);
  int topicsCountAfter = 0;
  for (auto iter : topicsAfter)
  {
    for (auto str : iter.second)
    {
      topicsCountAfter++;
      EXPECT_TRUE(str.find(modelName) == std::string::npos);
    }
  }
  EXPECT_EQ(topicsCountAfter, topicsCount+1);
}

TEST_P(ContactSensor, ModelRemoval)
{
  ModelRemoval(GetParam());
}

////////////////////////////////////////////////////////////////////////
// Test moving a model while in contact
// addresses a failure in pull request #1610 for simbody
////////////////////////////////////////////////////////////////////////
void ContactSensor::MoveTool(const std::string &_physicsEngine)
{
  Load("worlds/contact_sensors_multiple.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);

  const std::string modelName("sphere");
  const ignition::math::Vector3d pos(0, 0, 1.8);
  const ignition::math::Vector3d v30;
  const double radius = 0.5;
  SpawnSphere(modelName, pos, v30, v30, radius);

  // advertise on "~/model/modify"
  // so that we can move the sphere
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  // Step forward to allow the sphere to fall
  world->Step(200);

  // Try moving the model
  auto model = world->ModelByName(modelName);
  ASSERT_TRUE(model != NULL);

  auto pose = model->WorldPose();
  pose.Pos().X() += 0.2;
  pose.Pos().Y() += 0.2;

  msgs::Model msg;
  msg.set_name(modelName);
  msg.set_id(model->GetId());
  msgs::Set(msg.mutable_pose(), pose);
  modelPub->Publish(msg);

  while (pose != model->WorldPose())
  {
    world->Step(1);
    common::Time::MSleep(1);
  }

  world->Step(10);

  // it just needs to exit successfully in order to pass.
}

TEST_P(ContactSensor, MoveTool)
{
  MoveTool(GetParam());
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
        std::dynamic_pointer_cast<sensors::ContactSensor>(sensor1);
    ASSERT_TRUE(contactSensor1 != NULL);
  }

  {
    sensors::SensorPtr sensor2 = sensors::get_sensor(contactSensorName2);
    sensors::ContactSensorPtr contactSensor2 =
        std::dynamic_pointer_cast<sensors::ContactSensor>(sensor2);
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
  topicsExpected.sort();

  // Sleep to ensure transport topics are all advertised
  common::Time::MSleep(100);
  std::list<std::string> topics =
    transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  topics.sort();
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), topicsExpected.size());
  EXPECT_EQ(topics, topicsExpected);

  // We should expect them all to publish.
  for (auto const &topic : topics)
  {
    gzdbg << "Listening to " << topic << std::endl;
    g_messageCount = 0;
    transport::SubscriberPtr sub = this->node->Subscribe(topic,
      &ContactSensor::Callback, this);

    const unsigned int steps = 50;
    world->Step(steps);
    common::Time::MSleep(steps);
    EXPECT_GT(g_messageCount, steps / 2);
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
  physics::PhysicsEnginePtr physics = world->Physics();
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
      std::dynamic_pointer_cast<sensors::ContactSensor>(sensor01);

  ASSERT_TRUE(contactSensor01 != NULL);

  sensors::SensorPtr sensor02 = sensors::get_sensor(contactSensorName02);
  sensors::ContactSensorPtr contactSensor02 =
      std::dynamic_pointer_cast<sensors::ContactSensor>(sensor02);

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

  physics::ModelPtr contactModel01 = world->ModelByName(modelName01);
  physics::ModelPtr contactModel02 = world->ModelByName(modelName02);
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
    contacts01 = contactSensor01->Contacts();
    contacts02 = contactSensor02->Contacts();
    // gzdbg << "steps[" << steps
    //       << "] contacts01[" << contacts01.contact_size()
    //       << "] contacts02[" << contacts02.contact_size()
    //       << "] to be > 0\n";
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
    double mass = models[k]->GetLink()->GetInertial()->Mass();
    expectedForce = models[k]->GetLink()->WorldCoGPose().Rot().Inverse()
        * ignition::math::Vector3d(0, 0, (gravityZ * mass));
    expectedTorque = ignition::math::Vector3d::Zero;

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
        if (!ignition::math::equal(contacts[k].contact(i).position(j).z(), 1.0))
          continue;

        EXPECT_NEAR(contacts[k].contact(i).position(j).x(),
            models[k]->GetLink()->WorldCoGPose().Pos().X(), TOL);
        EXPECT_NEAR(contacts[k].contact(i).position(j).y(),
            models[k]->GetLink()->WorldCoGPose().Pos().Y(), TOL);

        EXPECT_NEAR(contacts[k].contact(i).normal(j).x(), 0, TOL);
        EXPECT_NEAR(contacts[k].contact(i).normal(j).y(), 0, TOL);
        EXPECT_NEAR(std::abs(contacts[k].contact(i).normal(j).z()), 1, TOL);

        if (body1)
        {
          actualForce.X() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().x();
          actualForce.Y() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().y();
          actualForce.Z() =
            contacts[k].contact(i).wrench(j).body_1_wrench().force().z();

          actualTorque.X() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().x();
          actualTorque.Y() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().y();
          actualTorque.Z() =
            contacts[k].contact(i).wrench(j).body_1_wrench().torque().z();
        }
        else
        {
          actualForce.X() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().x();
          actualForce.Y() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().y();
          actualForce.Z() =
            contacts[k].contact(i).wrench(j).body_2_wrench().force().z();

          actualTorque.X() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().x();
          actualTorque.Y() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().y();
          actualTorque.Z() =
            contacts[k].contact(i).wrench(j).body_2_wrench().torque().z();
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
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
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
      std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

  ASSERT_TRUE(contactSensor != NULL);

  sensors::SensorManager::Instance()->Init();
  sensors::SensorManager::Instance()->RunThreads();

  EXPECT_FALSE(contactSensor->IsActive());

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor->GetCollisionCount(), expectedColCount);

  contactSensor->SetActive(true);

  EXPECT_TRUE(contactSensor->IsActive());

  physics::ModelPtr contactModel = world->ModelByName(modelName);
  ASSERT_TRUE(contactModel != NULL);

  double gravityZ = -9.8;
  physics->SetGravity(ignition::math::Vector3d(0, 0, gravityZ));

  msgs::Contacts contacts;

  if (_physicsEngine == "_ode" || _physicsEngine == "bullet")
  {
    EXPECT_TRUE(physics->SetParam("iters", 100));
    if (_physicsEngine == "ode")
      EXPECT_TRUE(physics->SetParam("contact_max_correcting_vel", 0));
  }

  world->Step(1);

  // run simulation until contacts occur
  int steps = 2000;
  while (contacts.contact_size() == 0 && --steps > 0)
  {
    world->Step(1);
    contacts = contactSensor->Contacts();
  }

  EXPECT_GT(steps, 0);

  contacts = contactSensor->Contacts();

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
      if (contacts.contact(i).position(j).z() < 0.5)
        continue;

      if (body1)
      {
        actualTorque.X() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().x();
        actualTorque.Y() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().y();
        actualTorque.Z() =
          contacts.contact(i).wrench(j).body_1_wrench().torque().z();
      }
      else
      {
        actualTorque.X() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().x();
        actualTorque.Y() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().y();
        actualTorque.Z() =
          contacts.contact(i).wrench(j).body_2_wrench().torque().z();
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
