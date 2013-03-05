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
  Load("worlds/empty.world", false, _physicsEngine);

  std::stringstream contactModelStr01;
  std::string modelName01 = "contactModel01";
  std::string contactSensorName01 = "contactSensor01";
  math::Pose modelPose01(0, 0, 0.5, 0, 0, 0);

  std::stringstream contactModelStr02;
  std::string modelName02 = "contactModel02";
  std::string contactSensorName02 = "contactSensor02";
  math::Pose modelPose02(0, 0, 1.5, 0, 0, 0);

  SpawnBoxContactSensor(modelName01, contactSensorName01,
      math::Vector3(1, 1, 1), modelPose01.pos, modelPose01.rot.GetAsEuler());
  SpawnBoxContactSensor(modelName02, contactSensorName02,
      math::Vector3(1, 1, 1), modelPose02.pos, modelPose02.rot.GetAsEuler());

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

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  unsigned int expectedColCount = 1;
  EXPECT_EQ(contactSensor01->GetCollisionCount(), expectedColCount);
  EXPECT_EQ(contactSensor02->GetCollisionCount(), expectedColCount);

  contactSensor01->SetActive(true);
  contactSensor02->SetActive(true);

  EXPECT_TRUE(contactSensor01->IsActive());
  EXPECT_TRUE(contactSensor02->IsActive());

//  contactSensor01->Update(true);
//  contactSensor02->Update(true);

   // Get all the contacts.
  msgs::Contacts contacts;


  contacts = contactSensor01->GetContacts();

  gzerr << contacts.contact_size() << std::endl;

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    for (int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      gzerr << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      gzerr << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      gzerr << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      gzerr << "   Normal force 1: "
                << contacts.contact(i).normal(j).x() *
                   contacts.contact(i).wrench(j).body_1_force().x() +
                   contacts.contact(i).normal(j).y() *
                   contacts.contact(i).wrench(j).body_1_force().y() +
                   contacts.contact(i).normal(j).z() *
                   contacts.contact(i).wrench(j).body_1_force().z() << "\n";
    }
  }

//  msgs::Contact



//  EXPECT_TRUE(contactSensor01->GetCollisionContactCount(
//    contactSensor01->GetCollisionName(0)) >= 0);

//  EXPECT_TRUE(contactSensor02->GetCollisionContactCount(
//    contactSensor01->GetCollisionName(0)) >= 0);


//  gzerr << contactSensor01->GetCollisionName(0) << std::endl;;

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
