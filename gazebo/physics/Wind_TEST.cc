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
#include <memory>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

class WindTest : public ServerFixture
{
  /// \brief Callback for gztopic "~/response".
  public: void OnWindMsgResponse(ConstResponsePtr &_msg);

  /// \brief Get the global wind velocity, ignoring the entity.
  /// \param[in] _wind Pointer to the wind.
  /// \param[in] _entity Pointer to an entity at which location the wind
  /// velocity is to be calculated.
  /// \return Wind's velocity at entity's location.
  public: ignition::math::Vector3d LinearVel(
                  const physics::Wind *_wind,
                  const physics::Entity *_entity);

  /// \brief Test getting/setting wind parameters.
  public: void WindParam();

  /// \brief Test default wind parameters.
  public: void WindParamBool();

  /// \brief Test setting up function to compute the wind.
  public: void WindSetLinearVelFunc();

  /// \brief Incoming wind message.
  public: static msgs::Wind windPubMsg;

  /// \brief Received wind message.
  public: static msgs::Wind windResponseMsg;

  /// \brief Factor by which we multiply wind velocity.
  public: double windFactor;
};

msgs::Wind WindTest::windPubMsg;
msgs::Wind WindTest::windResponseMsg;

/////////////////////////////////////////////////
ignition::math::Vector3d WindTest::LinearVel(
                  const physics::Wind *_wind,
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

  windPubMsg.mutable_linear_velocity()->CopyFrom(
    msgs::Convert(ignition::math::Vector3d(0.7, 0.7, 0)));

  windPub->Publish(windPubMsg);

  // Wait 10 ms for the message to be processed
  common::Time::MSleep(10);

  msgs::Request *requestMsg = msgs::CreateRequest("wind_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (windResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);

  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_EQ(msgs::ConvertIgn(windResponseMsg.linear_velocity()),
            msgs::ConvertIgn(windPubMsg.linear_velocity()));

  // Test Wind::[GS]etParam()
  {
    physics::Wind &wind = world->Wind();
    ignition::math::Vector3d vel = boost::any_cast<ignition::math::Vector3d>(
      wind.Param("linear_velocity"));
    EXPECT_EQ(vel, msgs::ConvertIgn(windPubMsg.linear_velocity()));

    EXPECT_NO_THROW(wind.Param("fake_param_name"));
    EXPECT_NO_THROW(wind.SetParam("fake_param_name", 0));

    // Try SetParam with wrong type
    EXPECT_NO_THROW(wind.SetParam("linear_velocity", std::string("wrong")));
  }

  {
    // Test SetParam for non-implementation-specific parameters
    physics::Wind &wind = world->Wind();
    try
    {
      boost::any value;
      ignition::math::Vector3d vel(-1.03, 0, 0);
      EXPECT_TRUE(wind.SetParam("linear_velocity", vel));
      EXPECT_TRUE(wind.Param("linear_velocity", value));
      EXPECT_EQ(boost::any_cast<ignition::math::Vector3d>(value), vel);
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

  physics::Wind &wind = world->Wind();

  // Initialize to failure conditions
  boost::any value;

  // Test wind parameter(s)
  EXPECT_TRUE(wind.Param("linear_velocity", value));
  const ignition::math::Vector3d &vel =
    boost::any_cast<ignition::math::Vector3d>(value);
  EXPECT_NEAR(vel.X(), 0.0, 1e-6);
  EXPECT_NEAR(vel.Y(), 0.0, 1e-6);
  EXPECT_NEAR(vel.Z(), 0.0, 1e-6);

  EXPECT_FALSE(wind.Param("param_does_not_exist", value));
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

  physics::Wind &wind = world->Wind();

  // Double the speed
  this->windFactor = 2.0;

  wind.SetLinearVelFunc(std::bind(&WindTest::LinearVel, this,
        std::placeholders::_1, std::placeholders::_2));

  EXPECT_EQ(wind.WorldLinearVel(model.get()),
            this->LinearVel(&wind, model.get()));
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
