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
#include "physics/physics.hh"
#include "SimplePendulumIntegrator.hh"

using namespace gazebo;
class PhysicsTest : public ServerFixture
{
public: void EmptyWorld(std::string _worldFile);
public: void SimplePendulum(const std::string& _physicsEngine);
};

void PhysicsTest::EmptyWorld(std::string _worldFile)
{
  // Load an empty world
  Load(_worldFile, true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // simulate a couple seconds
  world->StepWorld(2000);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  Unload();
}

TEST_F(PhysicsTest, EmptyWorldODE)
{
  //EmptyWorld("worlds/empty.world");
}

#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, EmptyWorldRTQL8)
{
  //EmptyWorld("worlds/empty_rtql8.world");
}
#endif // HAVE_RTQL8

#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, SimplePendulumRTQL8_Loading)
{
  EmptyWorld("worlds/rtql8/simple_pendulum_rtql8.world");
}
#endif // HAVE_RTQL8

TEST_F(PhysicsTest, SimplePendulumODE)
{
  //SimplePendulum("ode");
}

#ifdef HAVE_BULLET
TEST_F(PhysicsTest, SimplePendulumRTQL8)
{
  //SimplePendulum("rtql8");
}
#endif  // HAVE_BULLET

void PhysicsTest::SimplePendulum(const std::string& _physicsEngine)
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

  double g = 9.81;
  double l = 10.0;
  double m = 10.0;

  double e_start;

  {
    // check velocity / energy
    math::Vector3 vel = link->GetWorldLinearVel();
    math::Pose pos = link->GetWorldPose();
    double pe = 9.81 * m * pos.pos.z;
    double ke = 0.5 * m * (vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
    e_start = pe + ke;
    // gzdbg << "total energy [" << e_start
    //       << "] pe[" << pe
    //       << "] ke[" << ke
    //       << "] p[" << pos.pos.z
    //       << "] v[" << vel
    //       << "]\n";
  }
  physicsEngine->SetStepTime(0.0001);
  physicsEngine->SetSORPGSIters(1000);

  {
    // test with global contact_max_correcting_vel at 0 as set by world file
    //   here we expect significant energy loss as the velocity correction
    //   is set to 0
    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->StepWorld(2000);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = 9.81 * m * pos.pos.z;
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

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
              PendulumAngle(g, l, 1.57079633, 0.0, world->GetSimTime().Double(),
                            0.000001) - 1.5707963);
        double actual_theta = joint->GetAngle(0).Radian();
        // gzdbg << "time [" << world->GetSimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->GetWorldPose()
        //       << "]\n";
        EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }



  {
    // test with global contact_max_correcting_vel at 100
    // here we expect much lower energy loss
    world->Reset();
    physicsEngine->SetContactMaxCorrectingVel(100);

    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->StepWorld(2000);
      {
        // check velocity / energy
        math::Vector3 vel = link->GetWorldLinearVel();
        math::Pose pos = link->GetWorldPose();
        double pe = 9.81 * m * pos.pos.z;
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

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
              PendulumAngle(g, l, 1.57079633, 0.0, world->GetSimTime().Double(),
                            0.000001) - 1.5707963);
        double actual_theta = joint->GetAngle(0).Radian();
        // gzdbg << "time [" << world->GetSimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->GetWorldPose()
        //       << "]\n";
        EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }
}

// Test for collision
#ifdef HAVE_RTQL8
TEST_F(PhysicsTest, Collision1RTQL8)
{
  //EmptyWorld("worlds/rtql8/collision1_rtql8.world");
}
#endif // HAVE_RTQL8

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
