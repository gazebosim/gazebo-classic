/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

class Issue3125Test : public ServerFixture
{
};

////////////////////////////////////////////////////////////////////////
TEST_F(Issue3125Test, SlowLoadingModelSpawn)
{
  Load("worlds/slow_loading_model_spawn.world", true);
  physics::WorldPtr world = physics::get_world();
  ASSERT_NE(world , nullptr);

  // Wait up to 20 seconds for model to spawn
  const std::string modelName = "slow_loading_model";
  auto model = world->ModelByName(modelName);
  int waitIteration = 0;
  const int maxWaitIterations = 200;
  while (!model && waitIteration < maxWaitIterations)
  {
    common::Time::MSleep(100);
    model = world->ModelByName(modelName);
    ++waitIteration;
  }
  ASSERT_NE(model, nullptr);

  // Initial pose
  const ignition::math::Pose3d expectedInitialPose(0, 0, 0.45, 0, 0, 0);
  EXPECT_EQ(expectedInitialPose, model->WorldPose());

  // Take 1 step and expect an initial velocity from the plugin
  world->Step(1);
  const ignition::math::Vector3d expectedInitialLinearVel(-1, -1, 4.9902);
  const ignition::math::Vector3d expectedInitialAngularVel(0.1, 5.0, 0.1);
  EXPECT_EQ(expectedInitialLinearVel, model->WorldLinearVel());
  EXPECT_EQ(expectedInitialAngularVel, model->WorldAngularVel());
}
