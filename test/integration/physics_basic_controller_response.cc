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

#include <map>
#include <string>
#include <vector>

#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/PID.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"

#define TOL 5e-5
using namespace gazebo;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  // the trikey model has three wheels oriented in different directions.
  // it caught a corner case in ODE where the inertia was being
  // truncated unequally based on cartesian orientation of the link.
  // hence this test is added to ensure we get the same dynamics behavior
  // regardless the orientation of the inertia matricies in world frame.
  public: void TrikeyWheelResponse(const std::string &_physicsEngine,
                                   const std::string &_worldFileName);
};

//////////////////////////////////////////////////
void PhysicsTest::TrikeyWheelResponse(const std::string &_physicsEngine,
                                      const std::string &_worldFileName)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "bullet dynamics not working as expected, issue #1144.\n";
    return;
  }
  if (_physicsEngine == "simbody" || _physicsEngine == "dart")
  {
    gzerr << "simbody and dart axis intepretation is wrong, see issue #1143.\n";
    return;
  }

  // Random seed is set to prevent brittle failures (gazebo issue #479)
  math::Rand::SetSeed(18420503);
  Load(_worldFileName, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // get model
  int i = 0;
  std::string modelName = "trikey";
  while (!this->HasEntity(modelName) && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model [" << modelName << "].");

  physics::ModelPtr model = world->GetModel(modelName);
  EXPECT_TRUE(model != NULL);

  // get joints
  physics::JointPtr joint1 = model->GetJoint("wheel1_joint");
  physics::JointPtr joint2 = model->GetJoint("wheel2_joint");
  physics::JointPtr joint3 = model->GetJoint("wheel3_joint");
  ASSERT_TRUE(joint1 != NULL);
  ASSERT_TRUE(joint2 != NULL);
  ASSERT_TRUE(joint3 != NULL);

  // setup position PID controllers for each joint
  const double pGain = 10;
  const double iGain = 0;
  const double dGain = 0;
  const double iMax = 0;
  const double iMin = 0;
  const double cmdMax = 0;
  const double cmdMin = 0;
  common::PID pid1(pGain, iGain, dGain, iMax, iMin, cmdMax, cmdMin);
  common::PID pid2(pGain, iGain, dGain, iMax, iMin, cmdMax, cmdMin);
  common::PID pid3(pGain, iGain, dGain, iMax, iMin, cmdMax, cmdMin);
  // set arbitrary target position
  const double cmdPos = 1.3;
  pid1.SetCmd(cmdPos);
  pid2.SetCmd(cmdPos);
  pid3.SetCmd(cmdPos);

  // step for some time and expect all three joints to behave
  // the same way (check joint velocity and position).
  double test_duration = 1.5;
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  int steps = test_duration/dt;

  double t0 = world->GetSimTime().Double();

  // for (int j = 0; j < 1000; ++j) //  for debug
  for (int i = 0; i < steps; ++i)
  {
    double pos1 = joint1->GetAngle(0).Radian();
    double pos2 = joint2->GetAngle(0).Radian();
    double pos3 = joint3->GetAngle(0).Radian();
    double error1 = pos1 - cmdPos;
    double error2 = pos2 - cmdPos;
    double error3 = pos3 - cmdPos;
    double force1 = pid1.Update(error1, dt);
    double force2 = pid2.Update(error2, dt);
    double force3 = pid3.Update(error3, dt);
    joint1->SetForce(0, force1);
    joint2->SetForce(0, force2);
    joint3->SetForce(0, force3);
    EXPECT_DOUBLE_EQ(world->GetSimTime().Double(), t0 + dt * i);
    EXPECT_NEAR(joint1->GetVelocity(0), joint2->GetVelocity(0), TOL);
    EXPECT_NEAR(joint1->GetVelocity(0), joint3->GetVelocity(0), TOL);
    EXPECT_NEAR(joint1->GetAngle(0).Radian(),
                joint2->GetAngle(0).Radian(), TOL);
    EXPECT_NEAR(joint1->GetAngle(0).Radian(),
                joint3->GetAngle(0).Radian(), TOL);
    world->Step(1);
  }
}

TEST_P(PhysicsTest, TrikeyWheelResponse)
{
  TrikeyWheelResponse(GetParam(), "worlds/inertia_ratio_reduction_test.world");
}

TEST_P(PhysicsTest, TrikeyWheelResponse2)
{
  TrikeyWheelResponse(GetParam(), "worlds/inertia_ratio_reduction_test2.world");
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
