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
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "test/ServerFixture.hh"

#define NEAR_TOL 2e-5

using namespace gazebo;
using namespace physics;

class BulletPhysics_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
/// Test setting and getting bullet physics params
TEST_F(BulletPhysics_TEST, PhysicsParam)
{
  std::string physicsEngineStr = "bullet";
  Load("worlds/empty.world", true, physicsEngineStr);
  WorldPtr world = get_world("default");
  ASSERT_TRUE(world != NULL);

  PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), physicsEngineStr);

  BulletPhysicsPtr bulletPhysics
      = boost::shared_static_cast<BulletPhysics>(physics);
  ASSERT_TRUE(bulletPhysics != NULL);

  std::string type = "sequential_impulse";
   int iters = 45;
  double sor = 1.2;
  double cfm = 0.3;
  double erp = 0.12;
  double contactSurfaceLayer = 0.02;

  // test setting/getting physics engine params
  bulletPhysics->SetParam(PhysicsEngine::SOLVER_TYPE, type);
  bulletPhysics->SetParam(PhysicsEngine::SOR_ITERS, iters);
  bulletPhysics->SetParam(PhysicsEngine::SOR, sor);
  bulletPhysics->SetParam(PhysicsEngine::GLOBAL_CFM, cfm);
  bulletPhysics->SetParam(PhysicsEngine::GLOBAL_ERP, erp);
  bulletPhysics->SetParam(PhysicsEngine::CONTACT_SURFACE_LAYER,
      contactSurfaceLayer);

  boost::any value;
  value = bulletPhysics->GetParam(PhysicsEngine::SOLVER_TYPE);
  std::string typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = bulletPhysics->GetParam(PhysicsEngine::SOR_ITERS);
  int itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = bulletPhysics->GetParam(PhysicsEngine::SOR);
  double sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = bulletPhysics->GetParam(PhysicsEngine::GLOBAL_CFM);
  double cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = bulletPhysics->GetParam(PhysicsEngine::GLOBAL_ERP);
  double erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = bulletPhysics->GetParam(PhysicsEngine::CONTACT_SURFACE_LAYER);
  double contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);

  // Set params to different values and verify the old values are correctly
  // replaced by the new ones.
  iters = 55;
  sor = 1.4;
  cfm = 0.1;
  erp = 0.22;
  contactSurfaceLayer = 0.03;

  bulletPhysics->SetParam(PhysicsEngine::SOLVER_TYPE, type);
  bulletPhysics->SetParam(PhysicsEngine::SOR_ITERS, iters);
  bulletPhysics->SetParam(PhysicsEngine::SOR, sor);
  bulletPhysics->SetParam(PhysicsEngine::GLOBAL_CFM, cfm);
  bulletPhysics->SetParam(PhysicsEngine::GLOBAL_ERP, erp);
  bulletPhysics->SetParam(PhysicsEngine::CONTACT_SURFACE_LAYER,
      contactSurfaceLayer);

  value = bulletPhysics->GetParam(PhysicsEngine::SOLVER_TYPE);
  typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = bulletPhysics->GetParam(PhysicsEngine::SOR_ITERS);
  itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = bulletPhysics->GetParam(PhysicsEngine::SOR);
  sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = bulletPhysics->GetParam(PhysicsEngine::GLOBAL_CFM);
  cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = bulletPhysics->GetParam(PhysicsEngine::GLOBAL_ERP);
  erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);
  value = bulletPhysics->GetParam(PhysicsEngine::CONTACT_SURFACE_LAYER);
  contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
