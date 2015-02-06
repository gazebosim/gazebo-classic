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

#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector3Stats.hh"
#include "gazebo/physics/physics.hh"
#include "test/integration/helper_physics_generator.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;

const double g_angle_y_tol = 0.21;
const double g_angle_z_tol = 0.21;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void InertiaRatioPendulum(const std::string &_physicsEngine);
  public: void InertiaRatioSphereStack(const std::string &_physicsEngine);
};

// Double pendulum with large inertia ratio and lateral gravity component
void PhysicsTest::InertiaRatioPendulum(const std::string &_physicsEngine)
{
  Load("worlds/inertia_ratio_pendulum.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // verify lateral gravity
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0.1, 0, -9.81));

  // get model
  physics::ModelPtr model = world->GetModel("inertia_ratio");
  ASSERT_TRUE(model != NULL);

  // get links
  physics::LinkPtr upperLink = model->GetLink("upper_link");
  physics::LinkPtr lowerLink = model->GetLink("lower_link");
  ASSERT_TRUE(upperLink != NULL);
  ASSERT_TRUE(lowerLink != NULL);

  math::Vector3Stats upperAngles;
  math::Vector3Stats lowerAngles;
  {
    const std::string statNames = "maxAbs";
    EXPECT_TRUE(upperAngles.InsertStatistics(statNames));
    EXPECT_TRUE(lowerAngles.InsertStatistics(statNames));
  }

  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < 3000; ++i)
  {
    world->Step(1);

    // Get statistics on link rotations
    upperAngles.InsertData(upperLink->GetWorldPose().rot.GetAsEuler());
    lowerAngles.InsertData(lowerLink->GetWorldPose().rot.GetAsEuler());
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("elapsedWallTime", elapsedTime.Double());
  this->Record("simTime", world->GetSimTime().Double());

  // Expect out of plane angles to fall within limits
  EXPECT_NEAR((upperAngles.Y().Map())["maxAbs"], 0.0, g_angle_y_tol);
  EXPECT_NEAR((upperAngles.Z().Map())["maxAbs"], 0.0, g_angle_z_tol);
  EXPECT_NEAR((lowerAngles.Y().Map())["maxAbs"], 0.0, g_angle_y_tol);
  EXPECT_NEAR((lowerAngles.Z().Map())["maxAbs"], 0.0, g_angle_z_tol);

  // Record statistics on pitch and yaw angles
  this->Record("upper_pitch_", upperAngles.Y());
  this->Record("lower_pitch_", lowerAngles.Y());
  this->Record("upper_yaw_", upperAngles.Z());
  this->Record("lower_yaw_", lowerAngles.Z());
}

TEST_P(PhysicsTest, InertiaRatioPendulum)
{
  InertiaRatioPendulum(GetParam());
}

// Stack of spheres with large inertia ratio under gravity
// Spheres under resting contact constraints
void PhysicsTest::InertiaRatioSphereStack(const std::string &_physicsEngine)
{
  Load("worlds/sphere_stack.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // verify lateral gravity
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0.1, 0, -9.81));

  // get models
  physics::ModelPtr sphere_1 = world->GetModel("sphere_1");
  physics::ModelPtr sphere_2 = world->GetModel("sphere_2");
  physics::ModelPtr sphere_3 = world->GetModel("sphere_3");
  physics::ModelPtr sphere_4 = world->GetModel("sphere_4");
  physics::ModelPtr sphere_5 = world->GetModel("sphere_5");
  ASSERT_TRUE(sphere_1 != NULL);
  ASSERT_TRUE(sphere_2 != NULL);
  ASSERT_TRUE(sphere_3 != NULL);
  ASSERT_TRUE(sphere_4 != NULL);
  ASSERT_TRUE(sphere_5 != NULL);

  // get links
  physics::LinkPtr link_1 = sphere_1->GetLink("upper_link");
  ASSERT_TRUE(link_1 != NULL);

  math::Vector3Stats upperAngles;
  math::Vector3Stats lowerAngles;
  {
    const std::string statNames = "MaxAbs,Rms";
    EXPECT_TRUE(upperAngles.InsertStatistics(statNames));
    EXPECT_TRUE(lowerAngles.InsertStatistics(statNames));
  }

  physics->SetRealTimeUpdateRate(0.0);
  common::Time startTime = common::Time::GetWallTime();
  for (int i = 0; i < 3000; ++i)
  {
    world->Step(1);

    // Get statistics on link rotations
    upperAngles.InsertData(link_1->GetWorldPose().rot.GetAsEuler());
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startTime;
  this->Record("elapsedWallTime", elapsedTime.Double());
  this->Record("simTime", world->GetSimTime().Double());

  // Expect out of plane angles to fall within limits
  EXPECT_NEAR((upperAngles.Y().Map())["maxAbs"], 0.0, g_angle_y_tol);
  EXPECT_NEAR((upperAngles.Z().Map())["maxAbs"], 0.0, g_angle_z_tol);
  EXPECT_NEAR((lowerAngles.Y().Map())["maxAbs"], 0.0, g_angle_y_tol);
  EXPECT_NEAR((lowerAngles.Z().Map())["maxAbs"], 0.0, g_angle_z_tol);

  // Record statistics on pitch and yaw angles
  this->Record("upper_pitch_", upperAngles.Y());
  this->Record("lower_pitch_", lowerAngles.Y());
  this->Record("upper_yaw_", upperAngles.Z());
  this->Record("lower_yaw_", lowerAngles.Z());
}

TEST_P(PhysicsTest, InertiaRatioSphereStack)
{
  InertiaRatioSphereStack(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
