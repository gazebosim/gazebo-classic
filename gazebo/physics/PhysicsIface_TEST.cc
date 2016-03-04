/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class PhysicsIfaceTest : public ServerFixture,
    public testing::WithParamInterface<const char*>
{
  /// \brief
  public: void RemoveWorldTest(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsIfaceTest::RemoveWorldTest(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
    return;

  // Load a world with some models
  // TODO: world with sensors?
  // TODO: sleep to make sure things are being created?
  this->Load("worlds/shapes.world", false, _physicsEngine);

  // Get world pointer
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get physics engine pointer
  auto physicsEngine = world->GetPhysicsEngine();
  ASSERT_TRUE(physicsEngine != NULL);

  // Check pointer use count
  gzdbg << "Use count: WorldPtr [" << world.use_count() <<
      "] PhysicsEnginePtr [" << physicsEngine.use_count() << "]" << std::endl;

  auto worldPtrCount = world.use_count();
  auto physicsEnginePtrCount = physicsEngine.use_count();

  EXPECT_GT(worldPtrCount, 1);
  EXPECT_GT(physicsEnginePtrCount, 1);

  // Remove world
  physics::remove_worlds();

  // Check pointer use count
  gzdbg << "Use count: WorldPtr [" << world.use_count() <<
      "] PhysicsEnginePtr [" << physicsEngine.use_count() << "]" << std::endl;

  EXPECT_LT(world.use_count(), worldPtrCount);
  EXPECT_LT(physicsEngine.use_count(), worldPtrCount);

  EXPECT_EQ(world.use_count(), 1);
  EXPECT_EQ(physicsEngine.use_count(), 1);

  // Release the last pointer
  world.reset();
  physicsEngine.reset();
}

/////////////////////////////////////////////////
TEST_P(PhysicsIfaceTest, RemoveWorldTest)
{
  RemoveWorldTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsIfaces, PhysicsIfaceTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
