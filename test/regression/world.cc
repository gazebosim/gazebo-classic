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
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

class WorldTest : public ServerFixture
{
  public: void OnWorldMsgResponse(ConstResponsePtr &_msg);
  public: void WorldParam();
  public: static msgs::World worldPubMsg;
  public: static msgs::World worldResponseMsg;

};

msgs::World WorldTest::worldPubMsg;
msgs::World WorldTest::worldResponseMsg;

/////////////////////////////////////////////////
void WorldTest::OnWorldMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == worldPubMsg.GetTypeName())
    worldResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void WorldTest::WorldParam()
{
  worldPubMsg.Clear();
  worldResponseMsg.Clear();

  Load("worlds/empty.world", false);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr worldNode;
  worldNode = transport::NodePtr(new transport::Node());
  worldNode->Init();

  transport::PublisherPtr worldPub
       = worldNode->Advertise<msgs::World>("~/world");
  transport::PublisherPtr requestPub
      = worldNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responseSub = worldNode->Subscribe("~/response",
      &WorldTest::OnWorldMsgResponse, this);

  worldPubMsg.set_max_step_size(0.01);
  worldPubMsg.set_real_time_update_rate(500);

  worldPub->Publish(worldPubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("world_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (worldResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);

  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_DOUBLE_EQ(worldResponseMsg.max_step_size(),
      worldPubMsg.max_step_size());
  EXPECT_DOUBLE_EQ(worldResponseMsg.real_time_update_rate(),
      worldPubMsg.real_time_update_rate());

  worldNode->Fini();
}

TEST_F(WorldTest, WorldParam)
{
  WorldParam();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
