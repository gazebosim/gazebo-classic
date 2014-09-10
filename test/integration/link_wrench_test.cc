/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/physics/Joint.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

class LinkWrenchTest : public ServerFixture,
                     public testing::WithParamInterface<const char*>
{
  /// \brief Link Wrench test.
  /// apply force and torque to a link, check its acceleration.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void LinkWrenchTest1(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void LinkWrenchTest::LinkWrenchTest1(const std::string &_physicsEngine)
{
  gzlog << "engine: [" << _physicsEngine << "]\n";
  // Load our force torque test world
  Load("worlds/link_wrench_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Start customizing test
  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model = world->GetModel("model");
  physics::LinkPtr link = model->GetLink("link");

  // this test assumes body frame axis is aligned with inertial
  // world frame axis to begin with

  // test about x-axis
  math::Vector3 torqueX(1.0, 0.0, 0.0);
  for (unsigned int i = 0; i < 10; ++i)
  {
    // initial velocity
    double wX0 = link->GetRelativeAngularVel().x;

    link->AddTorque(torqueX);
    // link->AddForce(math::Vector3(100.0, 0.0, 0.0));
    world->Step(1);

    // assuming constant torque is applied over dt, change in velocity
    // dw = dt * accel = dt * torque / Ixx
    double IXX = link->GetInertial()->GetIXX();
    double wX = wX0 + dt * torqueX.x / IXX;

    // check rotational velocity against torque added
    // gzerr << "wX0: [" << wX0
    //       << "] wX: [" << wX
    //       << "] t: [" << torqueX.x
    //       << "] I: [" << IXX
    //       << "] link wX: [" << link->GetRelativeAngularVel().x << "]\n";
    // getchar();
    EXPECT_DOUBLE_EQ(link->GetRelativeAngularVel().x, wX);
  }

  // if (_physicsEngine == simbody)
  // {
  //   // mysteriously, simbody failed with precision related error
  //   // for y and z axis after Reset.  See issue
  //   return;
  // }

  // test about y-axis
  model->Reset();
  math::Vector3 torqueY(0.0, 1.0, 0.0);
  for (unsigned int i = 0; i < 10; ++i)
  {
    // initial velocity
    double wY0 = link->GetRelativeAngularVel().y;

    link->AddTorque(torqueY);
    world->Step(1);

    // assuming constant torque is applied over dt, change in velocity
    // dw = dt * accel = dt * torque / Iyy
    double IYY = link->GetInertial()->GetIYY();
    double wY = wY0 + dt * torqueY.y / IYY;

    // check rotational velocity against torque added
    EXPECT_DOUBLE_EQ(link->GetRelativeAngularVel().y, wY);
  }

  // test about z-axis
  model->Reset();
  math::Vector3 torqueZ(0.0, 0.0, 1.0);
  for (unsigned int i = 0; i < 10; ++i)
  {
    // initial velocity
    double wZ0 = link->GetRelativeAngularVel().z;

    link->AddTorque(torqueZ);
    world->Step(1);

    // assuming constant torque is applied over dt, change in velocity
    // dw = dt * accel = dt * torque / Izz
    double IZZ = link->GetInertial()->GetIZZ();
    double wZ = wZ0 + dt * torqueZ.z / IZZ;

    // check rotational velocity against torque added
    EXPECT_DOUBLE_EQ(link->GetRelativeAngularVel().z, wZ);
  }

}

TEST_P(LinkWrenchTest, LinkWrenchTest1)
{
  LinkWrenchTest1(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, LinkWrenchTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
