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
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
using namespace physics;

class ODEPhysics_TEST : public ServerFixture
{
  public: void PhysicsMsgParam();
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: static msgs::Physics physicsPubMsg;
  public: static msgs::Physics physicsResponseMsg;
};

msgs::Physics ODEPhysics_TEST::physicsPubMsg;
msgs::Physics ODEPhysics_TEST::physicsResponseMsg;

/////////////////////////////////////////////////
/// Test setting and getting ode physics params
TEST_F(ODEPhysics_TEST, PhysicsParam)
{
  std::string physicsEngineStr = "ode";
  Load("worlds/empty.world", true, physicsEngineStr);
  WorldPtr world = get_world("default");
  ASSERT_TRUE(world != NULL);

  PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), physicsEngineStr);

  ODEPhysicsPtr odePhysics
      = boost::static_pointer_cast<ODEPhysics>(physics);
  ASSERT_TRUE(odePhysics != NULL);

  std::string type = "quick";
  int preconIters = 5;
  int iters = 45;
  double sor = 1.2;
  double cfm = 0.3;
  double erp = 0.12;
  double contactMaxCorrectingVel = 50;
  double contactSurfaceLayer = 0.02;

  // test setting/getting physics engine params
  odePhysics->SetParam("solver_type", type);
  odePhysics->SetParam("precon_iters", preconIters);
  odePhysics->SetParam("iters", iters);
  odePhysics->SetParam("sor", sor);
  odePhysics->SetParam("cfm", cfm);
  odePhysics->SetParam("erp", erp);
  odePhysics->SetParam("contact_max_correcting_vel", contactMaxCorrectingVel);
  odePhysics->SetParam("contact_surface_layer", contactSurfaceLayer);

  boost::any value;
  value = odePhysics->GetParam("solver_type");
  std::string typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = odePhysics->GetParam("precon_iters");
  int preconItersRet = boost::any_cast<int>(value);
  EXPECT_EQ(preconIters, preconItersRet);
  value = odePhysics->GetParam("iters");
  int itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = odePhysics->GetParam("sor");
  double sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = odePhysics->GetParam("cfm");
  double cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = odePhysics->GetParam("erp");
  double erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = odePhysics->GetParam("contact_max_correcting_vel");
  double contactMaxCorrectingVelRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel, contactMaxCorrectingVelRet);
  value = odePhysics->GetParam("contact_surface_layer");
  double contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);

  // verify against equivalent functions
  EXPECT_EQ(type, odePhysics->GetStepType());
  EXPECT_EQ(preconIters, odePhysics->GetSORPGSPreconIters());
  EXPECT_EQ(iters, odePhysics->GetSORPGSIters());
  EXPECT_DOUBLE_EQ(sor, odePhysics->GetSORPGSW());
  EXPECT_DOUBLE_EQ(cfm, odePhysics->GetWorldCFM());
  EXPECT_DOUBLE_EQ(erp, odePhysics->GetWorldERP());
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel,
      odePhysics->GetContactMaxCorrectingVel());
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, odePhysics->GetContactSurfaceLayer());

  // Set params to different values and verify the old values are correctly
  // replaced by the new ones.
  type = "world";
  preconIters = 10;
  iters = 55;
  sor = 1.4;
  cfm = 0.1;
  erp = 0.22;
  contactMaxCorrectingVel = 40;
  contactSurfaceLayer = 0.03;

  odePhysics->SetParam("solver_type", type);
  odePhysics->SetParam("precon_iters", preconIters);
  odePhysics->SetParam("iters", iters);
  odePhysics->SetParam("sor", sor);
  odePhysics->SetParam("cfm", cfm);
  odePhysics->SetParam("erp", erp);
  odePhysics->SetParam("contact_max_correcting_vel",
      contactMaxCorrectingVel);
  odePhysics->SetParam("contact_surface_layer",
      contactSurfaceLayer);

  value = odePhysics->GetParam("solver_type");
  typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = odePhysics->GetParam("iters");
  itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = odePhysics->GetParam("precon_iters");
  preconItersRet = boost::any_cast<int>(value);
  EXPECT_EQ(preconIters, preconItersRet);
  value = odePhysics->GetParam("sor");
  sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = odePhysics->GetParam("cfm");
  cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = odePhysics->GetParam("erp");
  erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = odePhysics->GetParam("contact_max_correcting_vel");
  contactMaxCorrectingVelRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel, contactMaxCorrectingVelRet);
  value = odePhysics->GetParam("contact_surface_layer");
  contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);

  EXPECT_EQ(type, odePhysics->GetStepType());
  EXPECT_EQ(preconIters, odePhysics->GetSORPGSPreconIters());
  EXPECT_EQ(iters, odePhysics->GetSORPGSIters());
  EXPECT_DOUBLE_EQ(sor, odePhysics->GetSORPGSW());
  EXPECT_DOUBLE_EQ(cfm, odePhysics->GetWorldCFM());
  EXPECT_DOUBLE_EQ(erp, odePhysics->GetWorldERP());
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel,
      odePhysics->GetContactMaxCorrectingVel());
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, odePhysics->GetContactSurfaceLayer());
}

/////////////////////////////////////////////////
void ODEPhysics_TEST::OnPhysicsMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == physicsPubMsg.GetTypeName())
    physicsResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void ODEPhysics_TEST::PhysicsMsgParam()
{
  physicsPubMsg.Clear();
  physicsResponseMsg.Clear();

  std::string physicsEngineStr = "ode";
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
      &ODEPhysics_TEST::OnPhysicsMsgResponse, this);

  physicsPubMsg.set_enable_physics(true);
  physicsPubMsg.set_max_step_size(0.001);
  physicsPubMsg.set_real_time_update_rate(800);
  physicsPubMsg.set_real_time_factor(1.1);
  physicsPubMsg.set_iters(60);
  physicsPubMsg.set_sor(1.5);
  physicsPubMsg.set_cfm(0.1);
  physicsPubMsg.set_erp(0.25);
  physicsPubMsg.set_contact_max_correcting_vel(10);
  physicsPubMsg.set_contact_surface_layer(0.01);

  physicsPubMsg.set_type(msgs::Physics::ODE);
  physicsPubMsg.set_solver_type("quick");

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
  EXPECT_EQ(physicsResponseMsg.solver_type(),
      physicsPubMsg.solver_type());
  EXPECT_EQ(physicsResponseMsg.enable_physics(),
      physicsPubMsg.enable_physics());
  EXPECT_EQ(physicsResponseMsg.iters(),
      physicsPubMsg.iters());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.sor(),
      physicsPubMsg.sor());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.cfm(),
      physicsPubMsg.cfm());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.contact_max_correcting_vel(),
      physicsPubMsg.contact_max_correcting_vel());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.contact_surface_layer(),
      physicsPubMsg.contact_surface_layer());

  phyNode->Fini();
}

/////////////////////////////////////////////////
TEST_F(ODEPhysics_TEST, PhysicsMsgParam)
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
