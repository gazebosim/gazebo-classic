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

#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PGSTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void SmoothingTest(const std::string &_physicsEngine);
  public: void WarmStartTest(const std::string &_physicsEngine);
};

void PGSTest::WarmStartTest(const std::string &_physicsEngine)
{
  // test warm starting with a pendulum
  Load("worlds/simple_pendulums.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  int i = 0;
  while (!this->HasEntity("model_1") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_1");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  EXPECT_TRUE(physicsEngine);
  physics::ModelPtr model = world->GetModel("model_1");
  EXPECT_TRUE(model);
  physics::LinkPtr link = model->GetLink("link_2");  // sphere link at end
  EXPECT_TRUE(link);
  physics::JointPtr joint = model->GetJoint("joint_0");
  EXPECT_TRUE(joint);

  double g = 9.81;
  double m = 10.0;

  double e_start;

  {
    // check velocity / energy
    math::Vector3 vel = link->GetWorldLinearVel();
    math::Pose pos = link->GetWorldPose();
    double pe = g * m * pos.pos.z;
    double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    e_start = pe + ke;
    // gzdbg << "total energy [" << e_start
    //       << "] pe[" << pe
    //       << "] ke[" << ke
    //       << "] p[" << pos.pos.z
    //       << "] v[" << vel
    //       << "]\n";
  }
  physicsEngine->SetMaxStepSize(0.0001);
  physicsEngine->SetSORPGSIters(1000);

  {
    // test with global contact_max_correcting_vel at 0 as set by world file
    //   here we expect significant energy loss as the velocity correction
    //   is set to 0
    int steps = 10000;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->Step(1);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = g * m * pos.pos.z;
        double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }
    }
  }
}
TEST_F(PGSTest, WarmStartTestODE)
{
  WarmStartTest("ode");
}

void PGSTest::SmoothingTest(const std::string &_physicsEngine)
{
  Load("worlds/simple_pendulums.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  int i = 0;
  while (!this->HasEntity("model_1") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_1");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  EXPECT_TRUE(physicsEngine);
  physics::ModelPtr model = world->GetModel("model_1");
  EXPECT_TRUE(model);
  physics::LinkPtr link = model->GetLink("link_2");  // sphere link at end
  EXPECT_TRUE(link);
  physics::JointPtr joint = model->GetJoint("joint_0");
  EXPECT_TRUE(joint);

  double g = 9.81;
  double m = 10.0;

  double e_start;

  {
    // check velocity / energy
    math::Vector3 vel = link->GetWorldLinearVel();
    math::Pose pos = link->GetWorldPose();
    double pe = g * m * pos.pos.z;
    double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    e_start = pe + ke;
    // gzdbg << "total energy [" << e_start
    //       << "] pe[" << pe
    //       << "] ke[" << ke
    //       << "] p[" << pos.pos.z
    //       << "] v[" << vel
    //       << "]\n";
  }
  physicsEngine->SetMaxStepSize(0.0001);
  physicsEngine->SetParam("iters", 1000);

  {
    // test with global contact_max_correcting_vel at 0 as set by world file
    //   here we expect significant energy loss as the velocity correction
    //   is set to 0
    int steps = 10000;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->Step(1);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = g * m * pos.pos.z;
        double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }
    }
  }
}
TEST_F(PGSTest, SmoothingTestODE)
{
  SmoothingTest("ode");
}


INSTANTIATE_TEST_CASE_P(PhysicsEngines, PGSTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
