/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <string.h>

#include "gazebo/physics/physics.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

const double g_big = 1e17;

class PhysicsCollisionTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Test Collision::GetBoundingBox.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetBoundingBox(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsCollisionTest::GetBoundingBox(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody" ||
      _physicsEngine == "dart")
  {
    gzerr << "Bounding boxes not yet working with "
          << _physicsEngine
          << ", see issue #1148"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check bounding box of ground plane
  {
    physics::ModelPtr model = world->GetModel("ground_plane");
    math::Box box = model->GetBoundingBox();
    EXPECT_LT(box.min.x, -g_big);
    EXPECT_LT(box.min.y, -g_big);
    EXPECT_LT(box.min.z, -g_big);
    EXPECT_GT(box.max.x, g_big);
    EXPECT_GT(box.max.y, g_big);
    EXPECT_DOUBLE_EQ(box.max.z, 0.0);
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, GetBoundingBox)
{
  GetBoundingBox(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsCollisionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
