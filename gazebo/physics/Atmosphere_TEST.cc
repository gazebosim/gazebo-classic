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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

class AtmosphereTest : public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  public: void OnAtmosphereMsgResponse(ConstResponsePtr &_msg);
  public: void AtmosphereParam(const std::string &_atmosphere);
  public: void AtmosphereParamBool(const std::string &_atmosphere);
  public: static msgs::Atmosphere atmospherePubMsg;
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

  Load("worlds/empty.world", false);
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

  msgs::Atmosphere_Type type;
  if (_atmosphere == "adiabatic")
    type = msgs::Atmosphere::ADIABATIC;
  else
    type = msgs::Atmosphere::ADIABATIC;
  atmospherePubMsg.set_type(type);
  atmospherePubMsg.set_temperature(0.01);
  atmospherePubMsg.set_pressure(500);
  atmospherePubMsg.set_mass_density(1.2);

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
    physics::AtmospherePtr atmosphere = world->GetAtmosphere();
    boost::any temperature = atmosphere->Param("temperature");
    EXPECT_DOUBLE_EQ(boost::any_cast<double>(temperature),
      atmospherePubMsg.temperature());

    EXPECT_NO_THROW(atmosphere->Param("fake_param_name"));
    EXPECT_NO_THROW(atmosphere->SetParam("fake_param_name", 0));

    // Try SetParam with wrong type
    EXPECT_NO_THROW(atmosphere->SetParam("temperature", std::string("wrong")));
  }

  {
    // Test SetParam for non-implementation-specific parameters
    physics::AtmospherePtr atmosphere = world->GetAtmosphere();
    try
    {
      boost::any value;
      double temperature = 0.02;
      double pressure = 0.03;
      double massDensity = 0.04;
      double temperatureGradient = 0.05;
      EXPECT_TRUE(atmosphere->SetParam("temperature", temperature));
      EXPECT_TRUE(atmosphere->Param("temperature", value));
      EXPECT_NEAR(boost::any_cast<double>(value), temperature, 1e-6);
      EXPECT_TRUE(atmosphere->SetParam("pressure", pressure));
      EXPECT_TRUE(atmosphere->Param("pressure", value));
      EXPECT_NEAR(boost::any_cast<double>(value), pressure, 1e-6);
      EXPECT_TRUE(atmosphere->SetParam("mass_density", massDensity));
      EXPECT_TRUE(atmosphere->Param("mass_density", value));
      EXPECT_NEAR(boost::any_cast<double>(value), massDensity, 1e-6);
      EXPECT_TRUE(atmosphere->SetParam("temperature_gradient",
                  temperatureGradient));
      EXPECT_TRUE(atmosphere->Param("temperature_gradient", value));
      EXPECT_NEAR(boost::any_cast<double>(value), temperatureGradient, 1e-6);
    }
    catch(boost::bad_any_cast &_e)
    {
      std::cout << "Bad any_cast in Atmosphere::SetParam test: " << _e.what()
                << std::endl;
      FAIL();
    }
  }

  atmosphereNode->Fini();
}

/////////////////////////////////////////////////
TEST_P(AtmosphereTest, AtmosphereParam)
{
  AtmosphereParam(GetParam());
}

/////////////////////////////////////////////////
void AtmosphereTest::AtmosphereParamBool
    (const std::string &_atmosphere)
{
  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::AtmospherePtr atmosphere = world->GetAtmosphere();

  // Initialize to failure conditions
  boost::any value;

  // Test shared atmosphere model parameter(s)
  EXPECT_TRUE(atmosphere->Param("temperature", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 288.15, 1e-6);
  EXPECT_TRUE(atmosphere->Param("mass_density", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 1.225, 1e-6);
  EXPECT_TRUE(atmosphere->Param("pressure", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 101325, 1e-6);
  EXPECT_TRUE(atmosphere->Param("temperature_gradient", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 0.0065, 1e-6);
  EXPECT_TRUE(atmosphere->Param("type", value));
  EXPECT_EQ(boost::any_cast<std::string>(value), _atmosphere);

  EXPECT_FALSE(atmosphere->Param("param_does_not_exist", value));
}

/////////////////////////////////////////////////
TEST_P(AtmosphereTest, AtmosphereParamBool)
{
  AtmosphereParamBool(GetParam());
}

INSTANTIATE_TEST_CASE_P(Atmospheres, AtmosphereTest,
                        ::testing::Values("adiabatic"));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
