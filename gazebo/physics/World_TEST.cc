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

#include <gtest/gtest.h>
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class World_TEST : public ServerFixture
{
  public: void GetEntityBelowPoint(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
/// \brief Test World::GetEntityBelowPoint
void World_TEST::GetEntityBelowPoint(const std::string &_physicsEngine)
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

    entity = world->GetEntityBelowPoint(pos);
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

TEST_F(World_TEST, GetEntityBelowPointODE)
{
  GetEntityBelowPoint("ode");
}

#ifdef HAVE_BULLET
TEST_F(World_TEST, GetEntityBelowPointBullet)
{
  GetEntityBelowPoint("bullet");
}
#endif  // HAVE_BULLET

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
