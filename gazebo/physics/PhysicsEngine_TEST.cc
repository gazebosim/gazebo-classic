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
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

class PhysicsEngineTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: void PhysicsEngineParam(const std::string &_physicsEngine);
  public: void PhysicsEngineGetParamBool(const std::string &_physicsEngine);
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
  transport::PublisherPtr requestPub
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
  else if (_physicsEngine == "simbody")
    type = msgs::Physics::SIMBODY;
  else
    type = msgs::Physics::ODE;
  physicsPubMsg.set_type(type);
  physicsPubMsg.set_max_step_size(0.01);
  physicsPubMsg.set_real_time_update_rate(500);
  physicsPubMsg.set_real_time_factor(1.2);

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

  // Test PhysicsEngine::[GS]etParam()
  {
    physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    boost::any dt = physics->GetParam("max_step_size");
    EXPECT_DOUBLE_EQ(boost::any_cast<double>(dt),
      physicsPubMsg.max_step_size());

    EXPECT_NO_THROW(physics->GetParam("fake_param_name"));
    EXPECT_NO_THROW(physics->SetParam("fake_param_name", 0));

    // Try SetParam with wrong type
    EXPECT_NO_THROW(physics->SetParam("iters", std::string("wrong")));
  }

  {
    // Test SetParam for non-implementation-specific parameters
    physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    try
    {
      boost::any value;
      double maxStepSize = 0.02;
      double realTimeUpdateRate = 0.03;
      double realTimeFactor = 0.04;
      ignition::math::Vector3d gravity(0, 0, 0);
      ignition::math::Vector3d magneticField(0.1, 0.1, 0.1);
      gazebo::math::Vector3 gravity2(0.2, 0.2, 0.2);
      gazebo::math::Vector3 magneticField2(0.3, 0.3, 0.3);
      gzdbg << "Set and Get max_step_size" << std::endl;
      EXPECT_TRUE(physics->SetParam("max_step_size", maxStepSize));
      EXPECT_TRUE(physics->GetParam("max_step_size", value));
      EXPECT_NEAR(boost::any_cast<double>(value), maxStepSize, 1e-6);
      gzdbg << "Set and Get real_time_update_rate" << std::endl;
      EXPECT_TRUE(physics->SetParam("real_time_update_rate",
          realTimeUpdateRate));
      EXPECT_TRUE(physics->GetParam("real_time_update_rate", value));
      EXPECT_NEAR(boost::any_cast<double>(value), realTimeUpdateRate, 1e-6);
      gzdbg << "Set and Get real_time_factor" << std::endl;
      EXPECT_TRUE(physics->SetParam("real_time_factor",
          realTimeFactor));
      EXPECT_TRUE(physics->GetParam("real_time_factor", value));
      EXPECT_NEAR(boost::any_cast<double>(value), realTimeFactor, 1e-6);
      gzdbg << "Set gravity as ignition::math::Vector3d" << std::endl;
      EXPECT_TRUE(physics->SetParam("gravity", gravity));
      EXPECT_TRUE(physics->GetParam("gravity", value));
      EXPECT_EQ(boost::any_cast<math::Vector3>(value), math::Vector3(gravity));
      gzdbg << "Set gravity as gazebo::math::Vector3" << std::endl;
      EXPECT_TRUE(physics->SetParam("gravity", math::Vector3(gravity2)));
      EXPECT_TRUE(physics->GetParam("gravity", value));
      EXPECT_EQ(boost::any_cast<math::Vector3>(value), math::Vector3(gravity2));
      gzdbg << "Set magnetic_field as ignition::math::Vector3d" << std::endl;
      EXPECT_TRUE(physics->SetParam("magnetic_field", magneticField));
      EXPECT_TRUE(physics->GetParam("magnetic_field", value));
      EXPECT_EQ(boost::any_cast<math::Vector3>(value),
                math::Vector3(magneticField));
      gzdbg << "Set magnetic_field as gazebo::math::Vector3" << std::endl;
      EXPECT_TRUE(physics->SetParam("magnetic_field",
                    math::Vector3(magneticField2)));
      EXPECT_TRUE(physics->GetParam("magnetic_field", value));
      EXPECT_EQ(boost::any_cast<math::Vector3>(value),
                math::Vector3(magneticField2));
    }
    catch(boost::bad_any_cast &_e)
    {
      std::cout << "Bad any_cast in PhysicsEngine::SetParam test: " << _e.what()
                << std::endl;
      FAIL();
    }
  }

  physicsNode->Fini();
}

/////////////////////////////////////////////////
TEST_P(PhysicsEngineTest, PhysicsEngineParam)
{
  PhysicsEngineParam(GetParam());
}

/////////////////////////////////////////////////
void PhysicsEngineTest::PhysicsEngineGetParamBool
    (const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();

  // Initialize to failure conditions
  boost::any value;

  // Test shared physics engine parameter(s)
  EXPECT_TRUE(physics->GetParam("gravity", value));
  EXPECT_EQ(boost::any_cast<math::Vector3>(value), math::Vector3(0, 0, -9.8));
  EXPECT_TRUE(physics->GetParam("max_step_size", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 0.001, 1e-6);
  EXPECT_TRUE(physics->GetParam("real_time_factor", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 1.0, 1e-6);
  EXPECT_TRUE(physics->GetParam("real_time_update_rate", value));
  EXPECT_NEAR(boost::any_cast<double>(value), 1000.0, 1e-6);
  EXPECT_TRUE(physics->GetParam("type", value));
  EXPECT_EQ(boost::any_cast<std::string>(value), _physicsEngine);

  if (_physicsEngine == "ode" || _physicsEngine == "bullet")
  {
    EXPECT_TRUE(physics->GetParam("iters", value));
    EXPECT_EQ(boost::any_cast<int>(value), 50);
  }
  else if (_physicsEngine == "dart")
  {
    gzwarn << "DARTPhysics::GetParam not yet implemented." << std::endl;
    return;
  }
  else if (_physicsEngine == "simbody")
  {
    EXPECT_TRUE(physics->GetParam("accuracy", value));
    EXPECT_NEAR(boost::any_cast<double>(value), 1e-3, 1e-6);
  }

  EXPECT_FALSE(physics->GetParam("param_does_not_exist", value));
}

/////////////////////////////////////////////////
TEST_P(PhysicsEngineTest, PhysicsEngineGetParamBool)
{
  PhysicsEngineGetParamBool(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsEngineTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
