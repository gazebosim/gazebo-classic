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
};

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
      = boost::shared_static_cast<ODEPhysics>(physics);
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
  odePhysics->SetParam(PhysicsEngine::SOLVER_TYPE, type);
  odePhysics->SetParam(PhysicsEngine::SOR_PRECON_ITERS, preconIters);
  odePhysics->SetParam(PhysicsEngine::SOR_ITERS, iters);
  odePhysics->SetParam(PhysicsEngine::SOR, sor);
  odePhysics->SetParam(PhysicsEngine::GLOBAL_CFM, cfm);
  odePhysics->SetParam(PhysicsEngine::GLOBAL_ERP, erp);
  odePhysics->SetParam(PhysicsEngine::CONTACT_MAX_CORRECTING_VEL,
      contactMaxCorrectingVel);
  odePhysics->SetParam(PhysicsEngine::CONTACT_SURFACE_LAYER,
      contactSurfaceLayer);

  boost::any value;
  value = odePhysics->GetParam(PhysicsEngine::SOLVER_TYPE);
  std::string typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = odePhysics->GetParam(PhysicsEngine::SOR_PRECON_ITERS);
  int preconItersRet = boost::any_cast<int>(value);
  EXPECT_EQ(preconIters, preconItersRet);
  value = odePhysics->GetParam(PhysicsEngine::SOR_ITERS);
  int itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = odePhysics->GetParam(PhysicsEngine::SOR);
  double sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = odePhysics->GetParam(PhysicsEngine::GLOBAL_CFM);
  double cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = odePhysics->GetParam(PhysicsEngine::GLOBAL_ERP);
  double erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = odePhysics->GetParam(PhysicsEngine::CONTACT_MAX_CORRECTING_VEL);
  double contactMaxCorrectingVelRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel, contactMaxCorrectingVelRet);
  value = odePhysics->GetParam(PhysicsEngine::CONTACT_SURFACE_LAYER);
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

  odePhysics->SetParam(PhysicsEngine::SOLVER_TYPE, type);
  odePhysics->SetParam(PhysicsEngine::SOR_PRECON_ITERS, preconIters);
  odePhysics->SetParam(PhysicsEngine::SOR_ITERS, iters);
  odePhysics->SetParam(PhysicsEngine::SOR, sor);
  odePhysics->SetParam(PhysicsEngine::GLOBAL_CFM, cfm);
  odePhysics->SetParam(PhysicsEngine::GLOBAL_ERP, erp);
  odePhysics->SetParam(PhysicsEngine::CONTACT_MAX_CORRECTING_VEL,
      contactMaxCorrectingVel);
  odePhysics->SetParam(PhysicsEngine::CONTACT_SURFACE_LAYER,
      contactSurfaceLayer);

  value = odePhysics->GetParam(PhysicsEngine::SOLVER_TYPE);
  typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = odePhysics->GetParam(PhysicsEngine::SOR_ITERS);
  itersRet = boost::any_cast<int>(value);
  value = odePhysics->GetParam(PhysicsEngine::SOR_PRECON_ITERS);
  preconItersRet = boost::any_cast<int>(value);
  EXPECT_EQ(preconIters, preconItersRet);
  EXPECT_EQ(iters, itersRet);
  value = odePhysics->GetParam(PhysicsEngine::SOR);
  sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = odePhysics->GetParam(PhysicsEngine::GLOBAL_CFM);
  cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = odePhysics->GetParam(PhysicsEngine::GLOBAL_ERP);
  erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = odePhysics->GetParam(PhysicsEngine::CONTACT_MAX_CORRECTING_VEL);
  contactMaxCorrectingVelRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel, contactMaxCorrectingVelRet);
  value = odePhysics->GetParam(PhysicsEngine::CONTACT_SURFACE_LAYER);
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
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
