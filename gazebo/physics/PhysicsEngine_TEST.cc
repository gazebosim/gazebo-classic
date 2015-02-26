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

#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;

class PhysicsEngineTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: void PhysicsEngineParam(const std::string &_physicsEngine);
  public: static msgs::Physics physicsPubMsg;
  public: static msgs::Physics physicsResponseMsg;
};

msgs::Physics PhysicsEngineTest::physicsPubMsg;
msgs::Physics PhysicsEngineTest::physicsResponseMsg;

/////////////////////////////////////////////////
void PhysicsEngineTest::OnPhysicsMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == physicsPubMsg.GetTypeName())
    physicsResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void PhysicsEngineTest::PhysicsEngineParam(const std::string &_physicsEngine)
{
  physicsPubMsg.Clear();
  physicsResponseMsg.Clear();

  Load("worlds/empty.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  transport::NodePtr physicsNode;
  physicsNode = transport::NodePtr(new transport::Node());
  physicsNode->Init();

  transport::PublisherPtr physicsPub
       = physicsNode->Advertise<msgs::Physics>("~/physics");
  transport::PublisherPtr localRequestPub
      = physicsNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responsePub = physicsNode->Subscribe("~/response",
      &PhysicsEngineTest::OnPhysicsMsgResponse, this);

  msgs::Physics_Type type;
  if (_physicsEngine == "ode")
    type = msgs::Physics::ODE;
  else if (_physicsEngine == "bullet")
    type = msgs::Physics::BULLET;
  else if (_physicsEngine == "dart")
    type = msgs::Physics::DART;
  else
    type = msgs::Physics::ODE;
  physicsPubMsg.set_type(type);
  physicsPubMsg.set_max_step_size(0.01);
  physicsPubMsg.set_real_time_update_rate(500);
  physicsPubMsg.set_real_time_factor(1.2);

  physicsPub->Publish(physicsPubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("physics_info", "");
  localRequestPub->Publish(*requestMsg);

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

  // Test PhysicsEngine::GetParam()
  {
    physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    boost::any dt = physics->GetParam("max_step_size");
    EXPECT_DOUBLE_EQ(boost::any_cast<double>(dt),
      physicsPubMsg.max_step_size());

    EXPECT_NO_THROW(physics->GetParam("fake_param_name"));
  }

  physicsNode->Fini();
}

/////////////////////////////////////////////////
TEST_P(PhysicsEngineTest, PhysicsEngineParam)
{
  PhysicsEngineParam(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsEngineTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
