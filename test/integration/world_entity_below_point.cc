/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;
class WorldTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  /// \brief Test World::GetEntityBelowPoint
  /// \param[in] _physicsEngine Type of physics engine to test.
  public: void GetEntityBelowPoint(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void WorldTest::GetEntityBelowPoint(const std::string &_physicsEngine)
{
  // Load in a world with lasers
  Load("worlds/shapes.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("box");
  modelNames.push_back("cylinder");
  modelNames.push_back("sphere");

  std::vector<std::string>::iterator iter;
  math::Vector3 pos, testPos;
  physics::ModelPtr model;
  physics::EntityPtr entity;
  for (iter = modelNames.begin(); iter != modelNames.end(); ++iter)
  {
    model = world->GetModel(*iter);
    ASSERT_TRUE(model != NULL);
    pos = model->GetWorldPose().pos;
    pos.z += 10;

    entity = world->GetEntityBelowPoint(pos);
    if (entity)
    {
      gzdbg << "hit: " << entity->GetScopedName()
            << ", expected: " << model->GetScopedName()
            << std::endl;
      EXPECT_EQ(entity->GetParentModel()->GetName(), model->GetName());
    }
    else
    {
      // Give a failure since we expected to get a return
      EXPECT_TRUE(entity != NULL);
    }

    testPos = pos;
    testPos.z = pos.z + 5;
    entity = world->GetEntityBelowPoint(testPos);
    if (entity)
    {
      gzdbg << "hit: " << entity->GetScopedName()
            << ", expected: " << model->GetScopedName()
            << std::endl;
      EXPECT_EQ(entity->GetParentModel(), model);
    }
    else
    {
      // Give a failure since we expected to get a return
      EXPECT_TRUE(entity != NULL);
    }
  }

  // Ground plane
  pos.Set(25, 25, 1);
  entity = world->GetEntityBelowPoint(pos);
  if (entity)
  {
    gzdbg << "hit: " << entity->GetScopedName()
          << ", expected: " << model->GetScopedName()
          << std::endl;
    EXPECT_EQ(entity->GetParentModel()->GetName(), "ground_plane");
  }
  else
  {
    // Give a failure since we expected to get a return
    EXPECT_TRUE(entity != NULL);
  }

  // Expect no hit
  pos.Set(25, 25, -1);
  entity = world->GetEntityBelowPoint(pos);
  EXPECT_TRUE(entity == NULL);
}

/////////////////////////////////////////////////
TEST_P(WorldTest, GetEntityBelowPoint)
{
  if (std::string(GetParam()) != "ode" &&
      std::string(GetParam()) != "bullet")
  {
    gzerr << "GetEntityBelowPoint not implemented for " << GetParam() << "\n";
  }
  else
  {
    GetEntityBelowPoint(GetParam());
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
