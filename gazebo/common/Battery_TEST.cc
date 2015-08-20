/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/common/Battery.hh"
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
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != NULL);

  EXPECT_DOUBLE_EQ(battery->Voltage(), 0.0);
  EXPECT_EQ(battery->PowerLoads().size(), (size_t)0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, AddConsumer)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != NULL);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, (uint32_t)0);
  EXPECT_EQ(battery->PowerLoads().size(), (size_t)1);

  battery->SetPowerLoad(consumerId, 5.0);

  double powerLoad = 0;
  EXPECT_TRUE(battery->PowerLoad(consumerId, powerLoad));
  EXPECT_DOUBLE_EQ(powerLoad, 5.0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, RemoveConsumer)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != NULL);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, (uint32_t)0);
  EXPECT_EQ(battery->PowerLoads().size(), (size_t)1);

  double powerLoad = 1.0;
  EXPECT_TRUE(battery->SetPowerLoad(consumerId, powerLoad));
  EXPECT_TRUE(battery->PowerLoad(consumerId, powerLoad));
  EXPECT_DOUBLE_EQ(powerLoad, 1.0);

  battery->RemoveConsumer(consumerId);
  EXPECT_EQ(battery->PowerLoads().size(), (size_t)0);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetPowerLoad)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != NULL);

  // Add two consumers
  uint32_t consumerId1 = battery->AddConsumer();
  uint32_t consumerId2 = battery->AddConsumer();
  EXPECT_EQ(battery->PowerLoads().size(), (size_t)2);

  // Set consumers power load
  double powerLoad1 = 1.0;
  double powerLoad2 = 2.0;
  EXPECT_TRUE(battery->SetPowerLoad(consumerId1, powerLoad1));
  EXPECT_TRUE(battery->SetPowerLoad(consumerId2, powerLoad2));

  // Check consumers power load
  EXPECT_TRUE(battery->PowerLoad(consumerId1, powerLoad1));
  EXPECT_DOUBLE_EQ(powerLoad1, 1.0);
  EXPECT_TRUE(battery->PowerLoad(consumerId2, powerLoad2));
  EXPECT_DOUBLE_EQ(powerLoad2, 2.0);
}

/// \brief A fixture class to help with updating the battery voltage.
class BatteryUpdateFixture
{
  /// \brief Update voltage by incrementing it.
  public: double Update(double _voltage,
                        const std::map<uint32_t, double> &/*_powerLoads*/)
          {
            return _voltage + this->step;
          }

  /// \brief Voltage amount to increment by.
  public: double step;
};

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetUpdateFunc)
{
  int N = 10;
  const double initVoltage = 12.0;

  std::ostringstream batteryStr;
  batteryStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<battery name='battery'>"
    <<     "<voltage>" << initVoltage << "</voltage>"
    <<   "</battery>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr batterySDF(new sdf::SDF);
  batterySDF->SetFromString(batteryStr.str());

  // Create the battery
  common::BatteryPtr battery(new common::Battery());
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
  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage);

  BatteryUpdateFixture fixture;
  fixture.step = -0.1;
  battery->SetUpdateFunc(std::bind(&BatteryUpdateFixture::Update,
        &fixture, std::placeholders::_1, std::placeholders::_2));

  for (int i = 0; i < N; ++i)
    battery->Update();

  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage + N * fixture.step);

  // Reinitialize the battery, and expect the same result
  battery->Init();

  for (int i = 0; i < N; ++i)
    battery->Update();

  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage + N * fixture.step);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
