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

#include <gtest/gtest.h>

#include "gazebo/ecs_core/EntityQueryResult.hh"
#include "gazebo/ecs_core/EntityQueryResultPrivate.hh"


namespace gazebo
{
namespace ecs_core
{
/// \brief class to abuse friendship
class EntityManager
{
  /// \brief Get private pointer from an EntityQueryResult
  public: std::shared_ptr<EntityQueryResultPrivate> GetImpl(
              const EntityQueryResult &_result)
    {
      return _result.impl;
    }
};
}
}

/////////////////////////////////////////////////
TEST(EntityQueryResult, InitialNoResults)
{
  gazebo::ecs_core::EntityQueryResult uut;
  EXPECT_EQ(0, uut.NumResults());
}

/////////////////////////////////////////////////
TEST(EntityQueryResult, TwoResults)
{
  gazebo::ecs_core::Entity e1 = 123;
  gazebo::ecs_core::Entity e2 = 456;
  gazebo::ecs_core::Entity e3 = 789;
  gazebo::ecs_core::EntityQueryResult uut;
  auto impl = gazebo::ecs_core::EntityManager().GetImpl(uut);

  impl->results.push_back(e1);
  impl->results.push_back(e2);
  ASSERT_EQ(2, uut.NumResults());

  // TODO an iterator would be more convenient
  bool foundE1 = false;
  bool foundE2 = false;
  bool foundE3 = false;
  for (int i = 0; i < 2; ++i)
  {
    if (uut.At(i) == e1)
      foundE1 = true;
    else if (uut.At(i) == e2)
      foundE2 = true;
    else
      foundE3 = true;
  }

  EXPECT_TRUE(foundE1);
  EXPECT_TRUE(foundE2);
  EXPECT_FALSE(foundE3);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
