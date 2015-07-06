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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Battery.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

using namespace gazebo;

class BatteryTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(BatteryTest, Construction)
{
  // Create the battery
  physics::BatteryPtr battery(new physics::Battery(physics::LinkPtr()));
  EXPECT_TRUE(battery != NULL);

  EXPECT_DOUBLE_EQ(battery->GetVoltage(), 0.0);
  EXPECT_EQ(battery->GetPowerLoads().size(), (size_t)0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, AddConsumer)
{
  // Create the battery
  physics::BatteryPtr battery(new physics::Battery(physics::LinkPtr()));
  EXPECT_TRUE(battery != NULL);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, (uint32_t)0);
  EXPECT_EQ(battery->GetPowerLoads().size(), (size_t)1);

  battery->SetPowerLoad(consumerId, 5.0);

  EXPECT_DOUBLE_EQ(battery->GetPowerLoad(consumerId), 5.0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, RemoveConsumer)
{
  // Create the battery
  physics::BatteryPtr battery(new physics::Battery(physics::LinkPtr()));
  EXPECT_TRUE(battery != NULL);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, (uint32_t)0);
  EXPECT_EQ(battery->GetPowerLoads().size(), (size_t)1);

  battery->SetPowerLoad(consumerId, 1.0);
  EXPECT_DOUBLE_EQ(battery->GetPowerLoad(consumerId), 1.0);

  battery->RemoveConsumer(consumerId);
  EXPECT_EQ(battery->GetPowerLoads().size(), (size_t)0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetPowerLoad)
{
  // Create the battery
  physics::BatteryPtr battery(new physics::Battery(physics::LinkPtr()));
  EXPECT_TRUE(battery != NULL);

  // Add two consumers
  uint32_t consumerId1 = battery->AddConsumer();
  uint32_t consumerId2 = battery->AddConsumer();
  EXPECT_EQ(battery->GetPowerLoads().size(), (size_t)2);

  // Set consumers power load
  battery->SetPowerLoad(consumerId1, 1.0);
  battery->SetPowerLoad(consumerId2, 2.0);

  // Check consumers power load
  EXPECT_DOUBLE_EQ(battery->GetPowerLoad(consumerId1), 1.0);
  EXPECT_DOUBLE_EQ(battery->GetPowerLoad(consumerId2), 2.0);
}

class BatteryUpdateFixture
{
  public: double Update(double _voltage,
                        const std::map<uint32_t, double> &/*_powerLoads*/)
          { return _voltage + this->step; }
  public: double step;
};

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetUpdateFunc)
{
  int N = 10;
  const double initVoltage = 12.0;
  const double initPowerLoad = 3.0;

  std::ostringstream batteryStr;
  batteryStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<battery>"
    <<     "<voltage>" << initVoltage << "</voltage>"
    <<     "<initial_power_load>" << initPowerLoad << "</initial_power_load>"
    <<   "</battery>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr batterySDF(new sdf::SDF);
  batterySDF->SetFromString(batteryStr.str());

  // Create the battery
  physics::BatteryPtr battery(new physics::Battery(physics::LinkPtr()));
  EXPECT_TRUE(battery != NULL);

  sdf::ElementPtr elem = batterySDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("battery");
  ASSERT_TRUE(elem != NULL);
  battery->Load(elem);

  battery->Init();

  BatteryUpdateFixture fixture;
  fixture.step = -0.1;
  battery->SetUpdateFunc(boost::bind(&BatteryUpdateFixture::Update,
                                     &fixture, _1, _2));

  for (int i = 0; i < N; ++i)
    gazebo::event::Events::worldUpdateEnd();

  EXPECT_DOUBLE_EQ(battery->GetVoltage(), initVoltage + N * fixture.step);

  // Reinitialize the battery, and expect the same result
  battery->Init();

  for (int i = 0; i < N; ++i)
    gazebo::event::Events::worldUpdateEnd();

  EXPECT_DOUBLE_EQ(battery->GetVoltage(), initVoltage + N * fixture.step);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
