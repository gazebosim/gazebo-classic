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

#include <gtest/gtest.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/test/ServerFixture.hh"

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
  double contactResidualSmoothing = 0.1;
  double contactSorScale = 1.0;
  bool threadPositionCorrection = true;
  bool experimentalRowReordering = true;
  double warmStartFactor = 1.0;
  int extraFrictionIterations = 15;

  // test setting/getting physics engine params
  EXPECT_TRUE(odePhysics->SetParam("solver_type", type));
  EXPECT_TRUE(odePhysics->SetParam("precon_iters", preconIters));
  EXPECT_TRUE(odePhysics->SetParam("iters", iters));
  EXPECT_TRUE(odePhysics->SetParam("sor", sor));
  EXPECT_TRUE(odePhysics->SetParam("cfm", cfm));
  EXPECT_TRUE(odePhysics->SetParam("erp", erp));
  EXPECT_TRUE(odePhysics->SetParam("contact_max_correcting_vel",
                                    contactMaxCorrectingVel));
  EXPECT_TRUE(odePhysics->SetParam("contact_surface_layer",
                                    contactSurfaceLayer));
  EXPECT_TRUE(odePhysics->SetParam("contact_residual_smoothing",
                                    contactResidualSmoothing));
  EXPECT_TRUE(odePhysics->SetParam("contact_sor_scale",
                                    contactSorScale));
  EXPECT_TRUE(odePhysics->SetParam("thread_position_correction",
                                    threadPositionCorrection));
  EXPECT_TRUE(odePhysics->SetParam("experimental_row_reordering",
                                    experimentalRowReordering));
  EXPECT_TRUE(odePhysics->SetParam("warm_start_factor",
                                    warmStartFactor));
  EXPECT_TRUE(odePhysics->SetParam("extra_friction_iterations",
                                    extraFrictionIterations));

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
  value = odePhysics->GetParam("contact_residual_smoothing");
  double contactResidualSmoothingRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactResidualSmoothing, contactResidualSmoothingRet);
  value = odePhysics->GetParam("contact_sor_scale");
  double contactSorScaleRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSorScale, contactSorScaleRet);
  value = odePhysics->GetParam("thread_position_correction");
  bool threadPositionCorrectionRet = boost::any_cast<bool>(value);
  EXPECT_EQ(threadPositionCorrection, threadPositionCorrectionRet);
  value = odePhysics->GetParam("experimental_row_reordering");
  bool experimentalRowReorderingRet = boost::any_cast<bool>(value);
  EXPECT_EQ(experimentalRowReordering, experimentalRowReorderingRet);
  value = odePhysics->GetParam("warm_start_factor");
  double warmStartFactorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(warmStartFactor, warmStartFactorRet);
  value = odePhysics->GetParam("extra_friction_iterations");
  int extraFrictionIterationsRet = boost::any_cast<int>(value);
  EXPECT_EQ(extraFrictionIterations, extraFrictionIterationsRet);

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
  contactResidualSmoothing = 0.09;
  contactSorScale = 0.9;
  threadPositionCorrection = false;
  experimentalRowReordering = false;
  warmStartFactor = 0.9;
  extraFrictionIterations = 14;

  EXPECT_TRUE(odePhysics->SetParam("solver_type", type));
  EXPECT_TRUE(odePhysics->SetParam("precon_iters", preconIters));
  EXPECT_TRUE(odePhysics->SetParam("iters", iters));
  EXPECT_TRUE(odePhysics->SetParam("sor", sor));
  EXPECT_TRUE(odePhysics->SetParam("cfm", cfm));
  EXPECT_TRUE(odePhysics->SetParam("erp", erp));
  EXPECT_TRUE(odePhysics->SetParam("contact_max_correcting_vel",
                                    contactMaxCorrectingVel));
  EXPECT_TRUE(odePhysics->SetParam("contact_surface_layer",
                                    contactSurfaceLayer));
  EXPECT_TRUE(odePhysics->SetParam("contact_residual_smoothing",
                                    contactResidualSmoothing));
  EXPECT_TRUE(odePhysics->SetParam("contact_sor_scale",
                                    contactSorScale));
  EXPECT_TRUE(odePhysics->SetParam("thread_position_correction",
                                    threadPositionCorrection));
  EXPECT_TRUE(odePhysics->SetParam("experimental_row_reordering",
                                    experimentalRowReordering));
  EXPECT_TRUE(odePhysics->SetParam("warm_start_factor",
                                    warmStartFactor));
  EXPECT_TRUE(odePhysics->SetParam("extra_friction_iterations",
                                    extraFrictionIterations));

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
  value = odePhysics->GetParam("contact_residual_smoothing");
  contactResidualSmoothingRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactResidualSmoothing, contactResidualSmoothingRet);
  value = odePhysics->GetParam("contact_sor_scale");
  contactSorScaleRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSorScale, contactSorScaleRet);
  value = odePhysics->GetParam("thread_position_correction");
  threadPositionCorrectionRet = boost::any_cast<bool>(value);
  EXPECT_EQ(threadPositionCorrection, threadPositionCorrectionRet);
  value = odePhysics->GetParam("experimental_row_reordering");
  experimentalRowReorderingRet = boost::any_cast<bool>(value);
  EXPECT_EQ(experimentalRowReordering, experimentalRowReorderingRet);
  value = odePhysics->GetParam("warm_start_factor");
  warmStartFactorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(warmStartFactor, warmStartFactorRet);
  value = odePhysics->GetParam("extra_friction_iterations");
  extraFrictionIterationsRet = boost::any_cast<int>(value);
  EXPECT_EQ(extraFrictionIterations, extraFrictionIterationsRet);

  EXPECT_EQ(type, odePhysics->GetStepType());
  EXPECT_EQ(preconIters, odePhysics->GetSORPGSPreconIters());
  EXPECT_EQ(iters, odePhysics->GetSORPGSIters());
  EXPECT_DOUBLE_EQ(sor, odePhysics->GetSORPGSW());
  EXPECT_DOUBLE_EQ(cfm, odePhysics->GetWorldCFM());
  EXPECT_DOUBLE_EQ(erp, odePhysics->GetWorldERP());
  EXPECT_DOUBLE_EQ(contactMaxCorrectingVel,
      odePhysics->GetContactMaxCorrectingVel());
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, odePhysics->GetContactSurfaceLayer());

  // Test dynamic MOI modification flag
  {
    std::vector<std::string> keys;
    const std::string key1 = "inertia_ratio_reduction";
    const std::string key2 = "use_dynamic_moi_rescaling";
    keys.push_back(key1);
    keys.push_back(key2);

    std::vector<bool> bools;
    bools.push_back(true);
    bools.push_back(false);

    // Set each keys with each flag value
    for (auto const &key : keys)
    {
      for (const bool &flag : bools)
      {
        gzdbg << "SetParam(" << key << ", " << flag << ")" << std::endl;
        EXPECT_TRUE(odePhysics->SetParam(key, flag));

        // Check both keys
        EXPECT_EQ(flag, boost::any_cast<bool>(odePhysics->GetParam(key1)));
        EXPECT_EQ(flag, boost::any_cast<bool>(odePhysics->GetParam(key2)));
      }
    }
  }

  // Test friction model
  {
    // Default value "pyramid_model"
    const std::string frictionModel = "pyramid_model";
    EXPECT_EQ(odePhysics->GetFrictionModel(), frictionModel);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("friction_model")));
    EXPECT_EQ(param, frictionModel);
  }

  {
    // Switch to "cone_model" using SetFrictionModel
    const std::string frictionModel = "cone_model";
    odePhysics->SetFrictionModel(frictionModel);
    EXPECT_EQ(odePhysics->GetFrictionModel(), frictionModel);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("friction_model")));
    EXPECT_EQ(param, frictionModel);
  }

  {
    // Switch to "box_model" using SetParam
    const std::string frictionModel = "box_model";
    odePhysics->SetParam("friction_model", frictionModel);
    EXPECT_EQ(odePhysics->GetFrictionModel(), frictionModel);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("friction_model")));
    EXPECT_EQ(param, frictionModel);
  }

  // Test world step solvers
  {
    // Default value "ODE_DANTZIG"
    const std::string worldSolverType = "ODE_DANTZIG";
    EXPECT_EQ(odePhysics->GetWorldStepSolverType(), worldSolverType);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("world_step_solver")));
    EXPECT_EQ(param, worldSolverType);
  }

  {
    // Switch to "DART_PGS" using SetWorldStepSolverType
    const std::string worldSolverType = "DART_PGS";
    odePhysics->SetWorldStepSolverType(worldSolverType);
    EXPECT_EQ(odePhysics->GetWorldStepSolverType(), worldSolverType);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("world_step_solver")));
    EXPECT_EQ(param, worldSolverType);
  }

  {
    // Switch to "BULLET_PGS" using SetParam
    const std::string worldSolverType = "BULLET_PGS";
    odePhysics->SetParam("world_step_solver", worldSolverType);
    EXPECT_EQ(odePhysics->GetWorldStepSolverType(), worldSolverType);
    std::string param;
    EXPECT_NO_THROW(param = boost::any_cast<std::string>(
      odePhysics->GetParam("world_step_solver")));
    EXPECT_EQ(param, worldSolverType);
  }
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
  /// \TODO: will not add below now, see asme branch for separation
  /// of physics params for different engines.
  // physicsPubMsg.set_contact_residual_smoothing(0.8);
  // physicsPubMsg.set_contact_sor_scale(0.7);
  // physicsPubMsg.set_thread_position_correction(true);
  // physicsPubMsg.set_experimental_row_reordering(true);
  // physicsPubMsg.set_warm_start_factor(0.6);
  // physicsPubMsg.set_extra_friction_iterations(89);

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
  /// \TODO: will not add below now, see asme branch for separation
  /// of physics params for different engines.
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.contact_residual_smoothing(),
  //     physicsPubMsg.contact_residual_smoothing());
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.contact_sor_scale(),
  //     physicsPubMsg.contact_sor_scale());
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.thread_position_correction(),
  //     physicsPubMsg.thread_position_correction());
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.experimental_row_reordering(),
  //     physicsPubMsg.experimental_row_reordering());
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.warm_start_factor(),
  //     physicsPubMsg.warm_start_factor());
  // EXPECT_DOUBLE_EQ(physicsResponseMsg.extra_friction_iterations(),
  //     physicsPubMsg.extra_friction_iterations());

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
