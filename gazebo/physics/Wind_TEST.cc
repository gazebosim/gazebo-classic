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

class WindTest : public ServerFixture
{
  public: void OnWindMsgResponse(ConstResponsePtr &_msg);
  public: ignition::math::Vector3d LinearVel(
                  const physics::WindPtr &_wind,
                  const physics::Entity *_entity);
  public: void WindParam();
  public: void WindParamBool();
  public: void WindSetLinearVelFunc();
  public: static msgs::Wind windPubMsg;
  public: static msgs::Wind windResponseMsg;
  public: double windFactor;
};

msgs::Wind WindTest::windPubMsg;
msgs::Wind WindTest::windResponseMsg;

/////////////////////////////////////////////////
ignition::math::Vector3d WindTest::LinearVel(
                  const physics::WindPtr &_wind,
                  const physics::Entity * /*_entity*/)
{
  return _wind->LinearVel() * this->windFactor;
}

/////////////////////////////////////////////////
void WindTest::OnWindMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == windPubMsg.GetTypeName())
    windResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void WindTest::WindParam()
{
  windPubMsg.Clear();
  windResponseMsg.Clear();

  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr windNode;
  windNode = transport::NodePtr(new transport::Node());
  windNode->Init();

  transport::PublisherPtr windPub
       = windNode->Advertise<msgs::Wind>("~/wind");
  transport::PublisherPtr requestPub
      = windNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responsePub = windNode->Subscribe("~/response",
      &WindTest::OnWindMsgResponse, this);

  windPubMsg.set_direction(45.0);
  windPubMsg.set_magnitude(1.03);

  windPub->Publish(windPubMsg);

  // Wait 10 ms for the message to be processed
  common::Time::MSleep(10);

  msgs::Request *requestMsg = msgs::CreateRequest("wind_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (windResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);

  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_DOUBLE_EQ(windResponseMsg.direction(),
      windPubMsg.direction());
  EXPECT_DOUBLE_EQ(windResponseMsg.magnitude(),
      windPubMsg.magnitude());

  // Test Wind::[GS]etParam()
  {
    physics::WindPtr wind = world->GetWind();
    boost::any direction = wind->Param("direction");
    EXPECT_DOUBLE_EQ(boost::any_cast<double>(direction),
      windPubMsg.direction());

    EXPECT_NO_THROW(wind->Param("fake_param_name"));
    EXPECT_NO_THROW(wind->SetParam("fake_param_name", 0));

    // Try SetParam with wrong type
    EXPECT_NO_THROW(wind->SetParam("direction", std::string("wrong")));
  }

  {
    // Test SetParam for non-implementation-specific parameters
    physics::WindPtr wind = world->GetWind();
    try
    {
      boost::any value;
      double direction = 3.14;
      double magnitude = 1.03;
      EXPECT_TRUE(wind->SetParam("direction", direction));
      EXPECT_TRUE(wind->Param("direction", value));
      EXPECT_NEAR(boost::any_cast<double>(value), direction, 1e-6);
      EXPECT_TRUE(wind->SetParam("magnitude", magnitude));
      EXPECT_TRUE(wind->Param("magnitude", value));
      EXPECT_NEAR(boost::any_cast<double>(value), magnitude, 1e-6);
    }
    catch(boost::bad_any_cast &_e)
    {
      std::cout << "Bad any_cast in Wind::SetParam test: " << _e.what()
                << std::endl;
      FAIL();
    }
  }

  windNode->Fini();
}

/////////////////////////////////////////////////
TEST_F(WindTest, WindParam)
{
  WindParam();
}

/////////////////////////////////////////////////
void WindTest::WindParamBool()
{
  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::WindPtr wind = world->GetWind();

  // Initialize to failure conditions
  boost::any value;

  // Test wind parameter(s)
  EXPECT_TRUE(wind->Param("direction", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 0.0, 1e-6);
  EXPECT_TRUE(wind->Param("magnitude", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 0.0, 1e-6);

  EXPECT_FALSE(wind->Param("param_does_not_exist", value));
}

/////////////////////////////////////////////////
TEST_F(WindTest, WindParamBool)
{
  WindParamBool();
}

/////////////////////////////////////////////////
void WindTest::WindSetLinearVelFunc()
{
  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Create a dummy model
  physics::ModelPtr model(new physics::Model(physics::BasePtr()));
  EXPECT_TRUE(model != NULL);

  physics::WindPtr wind = world->GetWind();

  // Double the speed
  this->windFactor = 2.0;

  wind->SetLinearVelFunc(std::bind(&WindTest::LinearVel, this,
        std::placeholders::_1, std::placeholders::_2));

  EXPECT_EQ(wind->WorldLinearVel(model.get()),
            this->LinearVel(wind, model.get()));
}

/////////////////////////////////////////////////
TEST_F(WindTest, WindSetLinearVelFunc)
{
  WindSetLinearVelFunc();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
