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

#include <algorithm>
#include <gtest/gtest.h>
#include <typeinfo>
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryPrivate.hh"

// Component Types for testing
struct TC1
{
  float itemOne;
};

struct TC2
{
  float itemOne;
  int itemTwo;
};

struct TC3
{
  float itemOne;
  int itemTwo;
  double itemThree;
};

// Type IDs for convenience
const std::size_t TC1HashCode = typeid(TC1).hash_code();
const std::size_t TC2HashCode = typeid(TC2).hash_code();
const std::size_t TC3HashCode = typeid(TC3).hash_code();

/////////////////////////////////////////////////
TEST(EntityQuery, AddSingleComponent)
{
  gazebo::ecs_core::EntityQuery uut;
  uut.AddComponent<TC1>();
  
  std::vector<std::size_t> types = uut.ComponentTypes();
  ASSERT_EQ(1, types.size());

  EXPECT_EQ(TC1HashCode, types[0]);
}

/////////////////////////////////////////////////
TEST(EntityQuery, AddTwoComponents)
{
  gazebo::ecs_core::EntityQuery uut;
  uut.AddComponent<TC1>();
  uut.AddComponent<TC2>();
  
  std::vector<std::size_t> types = uut.ComponentTypes();
  ASSERT_EQ(2, types.size());

  EXPECT_NE(types.end(), std::find(types.begin(), types.end(), TC1HashCode));
  EXPECT_NE(types.end(), std::find(types.begin(), types.end(), TC2HashCode));
  EXPECT_EQ(types.end(), std::find(types.begin(), types.end(), TC3HashCode));
}

/////////////////////////////////////////////////
TEST(EntityQuery, DuplicateComponent)
{
  gazebo::ecs_core::EntityQuery uut;
  uut.AddComponent<TC1>();
  uut.AddComponent<TC1>();
  
  std::vector<std::size_t> types = uut.ComponentTypes();
  ASSERT_EQ(1, types.size());
}

/////////////////////////////////////////////////
TEST(EntityQuery, ShallowCopyQuery)
{
  gazebo::ecs_core::EntityQuery uut;
  gazebo::ecs_core::EntityQuery uutCopy(uut);

  EXPECT_TRUE(uutCopy == uut);
}

/////////////////////////////////////////////////
TEST(EntityQuery, UnequalQueries)
{
  gazebo::ecs_core::EntityQuery uut;
  gazebo::ecs_core::EntityQuery uut2;

  EXPECT_FALSE(uut2 == uut);
}

/////////////////////////////////////////////////
TEST(EntityQuery, InitiallyNoResults)
{
  gazebo::ecs_core::EntityQuery uut;
  EXPECT_EQ(0, uut.Results()->NumResults());
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
