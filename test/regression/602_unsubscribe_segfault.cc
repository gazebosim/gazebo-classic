/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/common/PID.hh"
#include "test/ServerFixture.hh"
#include "test_config.h"

using namespace gazebo;
class Issue602Test : public ServerFixture
{
  /// \brief Test for Unsubscribe before delete subscriber.
  public: void UnsubscribeTest();

  /// \brief Callback for sensor subscribers in MultipleSensors test.
  /// \param[in] _msg World Statistics message.
  public: void Callback(const ConstContactsPtr &_msg);
};

unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void Issue602Test::Callback(const ConstContactsPtr &/*_msg*/)
{
  g_messageCount++;
}

/////////////////////////////////////////////////
TEST_F(Issue602Test, Unsubscribe)
{
  UnsubscribeTest();
}

/////////////////////////////////////////////////
void Issue602Test::UnsubscribeTest()
{
  Load("worlds/contact_sensors_multiple.world", true);
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

  auto topics = transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  EXPECT_FALSE(topics.empty());
  EXPECT_GE(topics.size(), 4u);

  // We should expect them all to publish.
  for (auto const &topic : topics)
  {
    gzdbg << "Listening to " << topic << std::endl;
    g_messageCount = 0;
    transport::SubscriberPtr sub = this->node->Subscribe(topic,
      &Issue602Test::Callback, this);

    const unsigned int steps = 50;
    world->Step(steps);
    common::Time::MSleep(steps);
    EXPECT_GT(g_messageCount, steps / 2);

    sub->Unsubscribe();
    common::Time::MSleep(steps);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
