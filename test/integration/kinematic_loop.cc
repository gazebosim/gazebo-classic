/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <cmath>

#include <gazebo/gazebo_config.h>
#ifdef HAVE_DART
#include <dart/config.hpp>
#endif

#include <ignition/math/Vector3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

const double g_tolerance = 1e-2;

class KinematicLoopTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
  /// \brief Spawn a 3-link kinematic chain anchored to the world.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _solverEngine Type of solver to use (optional).
  public: void AnchoredLoop(const std::string &_physicsEngine,
                            const std::string &_solverType = "");

  /// \brief Spawn a 4-link kinematic loop and drop it.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void FreeLoop(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void KinematicLoopTest::AnchoredLoop(const std::string &_physicsEngine,
                                     const std::string &_solverType)
{
  Load("worlds/anchored_loop.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  auto model = world->ModelByName("anchored_loop");
  ASSERT_TRUE(model != nullptr);

  // Unthrottle update rate
  auto physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  physics->SetRealTimeUpdateRate(0.0);

  // Set solver_type
  if (!_solverType.empty())
  {
    physics->SetParam("solver_type", _solverType);
    EXPECT_EQ(_solverType,
        boost::any_cast<std::string>(physics->GetParam("solver_type")));
  }

  // Simulate 15s
  const double dt = physics->GetMaxStepSize();
  world->Step(15.0/dt);

  auto joint1 = model->GetJoint("joint1");
  auto joint2 = model->GetJoint("joint2");
  auto joint3 = model->GetJoint("joint3");
  auto joint4 = model->GetJoint("joint4");
  ASSERT_TRUE(joint1 != nullptr);
  ASSERT_TRUE(joint2 != nullptr);
  ASSERT_TRUE(joint3 != nullptr);
  ASSERT_TRUE(joint4 != nullptr);

  // Should be at rest
  EXPECT_NEAR(joint1->GetVelocity(0), 0.0, g_tolerance);
  EXPECT_NEAR(joint2->GetVelocity(0), 0.0, g_tolerance);
  EXPECT_NEAR(joint3->GetVelocity(0), 0.0, g_tolerance);
  EXPECT_NEAR(joint4->GetVelocity(0), 0.0, g_tolerance);

  // Approximate expected configuration
  EXPECT_NEAR(joint1->Position(), 0.61, 0.1);
  EXPECT_NEAR(joint2->Position(), -1.25, 0.1);
  EXPECT_NEAR(joint3->Position(), 0.82, 0.1);
  EXPECT_NEAR(joint4->Position(), 0.19, 0.1);
}

/////////////////////////////////////////////////
void KinematicLoopTest::FreeLoop(const std::string &_physicsEngine)
{
  Load("worlds/free_loop.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);
  auto model = world->ModelByName("free_loop");
  ASSERT_TRUE(model != nullptr);

  // Unthrottle update rate
  auto physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  physics->SetRealTimeUpdateRate(0.0);

  // Simulate 15s
  const double dt = physics->GetMaxStepSize();
  world->Step(15.0/dt);

  auto joint1 = model->GetJoint("joint_0_1");
  auto joint2 = model->GetJoint("joint_1_2");
  auto joint3 = model->GetJoint("joint_2_3");
  auto joint4 = model->GetJoint("joint_3_0");
  ASSERT_TRUE(joint1 != nullptr);
  ASSERT_TRUE(joint2 != nullptr);
  ASSERT_TRUE(joint3 != nullptr);
  ASSERT_TRUE(joint4 != nullptr);

  // Approximate expected pose (has not flipped onto its side)
  ignition::math::Vector3d zHat(0, 0, 1);
  ignition::math::Vector3d normal = model->WorldPose().Rot()*zHat;
  EXPECT_NEAR(normal.Dot(zHat), 0.0, 0.7071);

  // Should be approximately at rest
  EXPECT_NEAR(model->WorldLinearVel().Length(), 0.0, 0.5);
  EXPECT_NEAR(model->WorldAngularVel().Length(), 0.0, 0.5);
  EXPECT_NEAR(joint1->GetVelocity(0), 0.0, 0.1);
  EXPECT_NEAR(joint2->GetVelocity(0), 0.0, 0.1);
  EXPECT_NEAR(joint3->GetVelocity(0), 0.0, 0.1);
  EXPECT_NEAR(joint4->GetVelocity(0), 0.0, 0.1);

  // Expected configuration
  double dAngle = IGN_PI/2.0 - 2.0*atan2(0.1, 0.8);
  if (_physicsEngine == "simbody")
  {
    gzerr << "Self-collision not implemented for " << _physicsEngine << ".\n";
    // Joint limit
    dAngle = 1.4;
  }
  EXPECT_NEAR(joint1->Position(), dAngle, g_tolerance);
  EXPECT_NEAR(joint2->Position(), -dAngle, g_tolerance);
  EXPECT_NEAR(joint3->Position(), dAngle, g_tolerance);
  EXPECT_NEAR(joint4->Position(), -dAngle, g_tolerance);
}

/////////////////////////////////////////////////
TEST_P(KinematicLoopTest, AnchoredLoop)
{
  AnchoredLoop(GetParam());
}

/////////////////////////////////////////////////
#ifdef HAVE_DART
#if DART_MAJOR_MINOR_VERSION_AT_LEAST(6, 8)
TEST_F(KinematicLoopTest, AnchoredLoopDARTDantzig)
{
  AnchoredLoop("dart", "dantzig");
}
#endif
#endif

/////////////////////////////////////////////////
TEST_P(KinematicLoopTest, FreeLoop)
{
  FreeLoop(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, KinematicLoopTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
