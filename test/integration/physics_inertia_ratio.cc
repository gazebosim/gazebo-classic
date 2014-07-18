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

#include <map>
#include <string>
#include <vector>

#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "test/integration/helper_physics_generator.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;

const double g_angle_y_tol = 0.2;
const double g_angle_z_tol = 0.2;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void InertiaRatioPendulum(const std::string &_physicsEngine);
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

  for (int i = 0; i < 3000; ++i)
  {
    world->StepWorld(1);

    // Check out of plane angles
    {
      math::Pose pose;
      pose = upperLink->GetWorldPose();
      EXPECT_NEAR(pose.rot.y, 0.0, g_angle_y_tol);
      EXPECT_NEAR(pose.rot.z, 0.0, g_angle_z_tol);
    }

    {
      math::Pose pose;
      pose = lowerLink->GetWorldPose();
      EXPECT_NEAR(pose.rot.y, 0.0, g_angle_y_tol);
      EXPECT_NEAR(pose.rot.z, 0.0, g_angle_z_tol);
    }
  }
}

TEST_P(PhysicsTest, InertiaRatioPendulum)
{
  InertiaRatioPendulum(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
