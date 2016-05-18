/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
using namespace physics;

class PluginPhysics_TEST : public ServerFixture
{
  public: void PhysicsMsgParam();
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: static msgs::Physics physicsPubMsg;
  public: static msgs::Physics physicsResponseMsg;
};

msgs::Physics PluginPhysics_TEST::physicsPubMsg;
msgs::Physics PluginPhysics_TEST::physicsResponseMsg;

/////////////////////////////////////////////////
/// Test setting and getting plugin physics params
TEST_F(PluginPhysics_TEST, PhysicsParam)
{
  std::string physicsEngineStr = "plugin";
  Load("worlds/empty.world", true, physicsEngineStr);
  WorldPtr world = get_world("default");
  ASSERT_TRUE(world != NULL);

  PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), physicsEngineStr);

  PluginPhysicsPtr pluginPhysics
      = boost::static_pointer_cast<PluginPhysics>(physics);
  ASSERT_TRUE(pluginPhysics != NULL);

}

/////////////////////////////////////////////////
void PluginPhysics_TEST::OnPhysicsMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == physicsPubMsg.GetTypeName())
    physicsResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void PluginPhysics_TEST::PhysicsMsgParam()
{
  physicsPubMsg.Clear();
  physicsResponseMsg.Clear();

  std::string physicsEngineStr = "plugin";
  Load("worlds/empty.world", false, physicsEngineStr);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
  ASSERT_TRUE(engine != NULL);

  transport::NodePtr phyNode;
  phyNode = transport::NodePtr(new transport::Node());
  phyNode->Init();

  transport::PublisherPtr physicsPub
       = phyNode->Advertise<msgs::Physics>("~/physics");
  transport::PublisherPtr requestPub
      = phyNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responseSub = phyNode->Subscribe("~/response",
      &PluginPhysics_TEST::OnPhysicsMsgResponse, this);

  physicsPubMsg.set_enable_physics(true);
  physicsPubMsg.set_max_step_size(0.001);
  physicsPubMsg.set_real_time_update_rate(800);
  physicsPubMsg.set_real_time_factor(1.1);
  physicsPubMsg.set_iters(60);

  physicsPubMsg.set_type(msgs::Physics::PLUGIN);

  physicsPub->Publish(physicsPubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("physics_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (physicsResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);
  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_DOUBLE_EQ(physicsResponseMsg.max_step_size(),
      physicsPubMsg.max_step_size());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.real_time_update_rate(),
      physicsPubMsg.real_time_update_rate());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.real_time_factor(),
      physicsPubMsg.real_time_factor());
  EXPECT_EQ(physicsResponseMsg.enable_physics(),
      physicsPubMsg.enable_physics());
  phyNode->Fini();
}

/////////////////////////////////////////////////
TEST_F(PluginPhysics_TEST, PhysicsMsgParam)
{
  PhysicsMsgParam();
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
