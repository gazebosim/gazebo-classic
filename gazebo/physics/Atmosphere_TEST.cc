/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

class AtmosphereTest : public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  /// \brief Callback for gztopic "~/response".
  /// \param[in] _msg Message received from topic.
  public: void OnAtmosphereMsgResponse(ConstResponsePtr &_msg);

  /// \brief Test getting/setting atmosphere model parameters.
  /// \param[in] _atmosphere Name of the atmosphere model.
  public: void AtmosphereParam(const std::string &_atmosphere);

  /// \brief Test default atmosphere model parameters
  /// \param[in] _atmosphere Name of the atmosphere model.
  public: void AtmosphereParamBool(const std::string &_atmosphere);

  /// \brief Incoming atmosphere message.
  public: static msgs::Atmosphere atmospherePubMsg;

  /// \brief Received atmosphere message.
  public: static msgs::Atmosphere atmosphereResponseMsg;
};

msgs::Atmosphere AtmosphereTest::atmospherePubMsg;
msgs::Atmosphere AtmosphereTest::atmosphereResponseMsg;

/////////////////////////////////////////////////
void AtmosphereTest::OnAtmosphereMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == atmospherePubMsg.GetTypeName())
    atmosphereResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void AtmosphereTest::AtmosphereParam(const std::string &_atmosphere)
{
  atmospherePubMsg.Clear();
  atmosphereResponseMsg.Clear();

  this->Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr atmosphereNode;
  atmosphereNode = transport::NodePtr(new transport::Node());
  atmosphereNode->Init();

  transport::PublisherPtr atmospherePub
       = atmosphereNode->Advertise<msgs::Atmosphere>("~/atmosphere");
  transport::PublisherPtr requestPub
      = atmosphereNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responsePub = atmosphereNode->Subscribe("~/response",
      &AtmosphereTest::OnAtmosphereMsgResponse, this);

  ASSERT_EQ(_atmosphere, "adiabatic");
  atmospherePubMsg.set_type(msgs::Atmosphere::ADIABATIC);
  atmospherePubMsg.set_temperature(0.01);
  atmospherePubMsg.set_pressure(500);
  atmospherePubMsg.set_mass_density(174.18084087484144);

  atmospherePub->Publish(atmospherePubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("atmosphere_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (atmosphereResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);

  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_DOUBLE_EQ(atmosphereResponseMsg.temperature(),
      atmospherePubMsg.temperature());
  EXPECT_DOUBLE_EQ(atmosphereResponseMsg.pressure(),
      atmospherePubMsg.pressure());
  EXPECT_DOUBLE_EQ(atmosphereResponseMsg.mass_density(),
      atmospherePubMsg.mass_density());

  // Test Atmosphere::[GS]etParam()
  {
    physics::Atmosphere &atmosphere = world->Atmosphere();
    double temperature = atmosphere.Temperature();
    EXPECT_DOUBLE_EQ(temperature, atmospherePubMsg.temperature());
  }

  // Test SetParam for non-implementation-specific parameters
  physics::Atmosphere &atmosphere = world->Atmosphere();
  double temperature = 0.02;
  double pressure = 0.03;
  double temperatureGradient = 0.05;
  atmosphere.SetTemperature(temperature);
  EXPECT_NEAR(atmosphere.Temperature(), temperature, 1e-6);
  atmosphere.SetPressure(pressure);
  EXPECT_NEAR(atmosphere.Pressure(), pressure, 1e-6);
  atmosphere.SetTemperatureGradient(temperatureGradient);
  EXPECT_NEAR(atmosphere.TemperatureGradient(), temperatureGradient, 1e-6);

  atmosphereNode->Fini();
}

/////////////////////////////////////////////////
TEST_P(AtmosphereTest, AtmosphereParam)
{
  AtmosphereParam(this->GetParam());
}

/////////////////////////////////////////////////
void AtmosphereTest::AtmosphereParamBool
    (const std::string &_atmosphere)
{
  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::Atmosphere &atmosphere = world->Atmosphere();

  // Test shared atmosphere model parameter(s)
  EXPECT_NEAR(atmosphere.Temperature(), 288.15, 1e-6);
  EXPECT_NEAR(atmosphere.MassDensity(), 1.2249782197913108, 1e-6);
  EXPECT_NEAR(atmosphere.Pressure(), 101325, 1e-6);
  EXPECT_NEAR(atmosphere.TemperatureGradient(), -0.0065, 1e-6);
  EXPECT_EQ(atmosphere.Type(), _atmosphere);

  // Test atmosphere model parameters at a given altitude
  if (_atmosphere == "adiabatic")
  {
    double altitude = 1000;
    EXPECT_NEAR(atmosphere.Temperature(altitude), 281.64999999999998, 1e-6);
    EXPECT_NEAR(atmosphere.MassDensity(altitude), 1.1117154882870524, 1e-6);
    EXPECT_NEAR(atmosphere.Pressure(altitude), 89882.063292207444, 1e-6);
  }
}

/////////////////////////////////////////////////
TEST_P(AtmosphereTest, AtmosphereParamBool)
{
  AtmosphereParamBool(this->GetParam());
}

INSTANTIATE_TEST_CASE_P(Atmospheres, AtmosphereTest,
                        ::testing::Values("adiabatic"));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
