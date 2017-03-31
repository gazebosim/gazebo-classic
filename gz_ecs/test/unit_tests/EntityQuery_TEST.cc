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
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/EntityQuery.hh"

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


/////////////////////////////////////////////////
TEST(EntityQuery, AddSingleComponent)
{
  gazebo::ecs::EntityQuery uut;
  uut.AddComponent("TC1");
  
  std::set<gazebo::ecs::ComponentType> types = uut.ComponentTypes();
  ASSERT_EQ(1, types.size());

  EXPECT_EQ(types.begin() , std::find(types.begin(), types.end(),
        gazebo::ecs::ComponentFactory::Type<TC1>()));
}

/////////////////////////////////////////////////
TEST(EntityQuery, AddTwoComponents)
{
  gazebo::ecs::EntityQuery uut;
  uut.AddComponent("TC1");
  uut.AddComponent("TC2");
  
  std::set<gazebo::ecs::ComponentType> types = uut.ComponentTypes();
  ASSERT_EQ(2, types.size());

  EXPECT_NE(types.end(), std::find(types.begin(), types.end(),
        gazebo::ecs::ComponentFactory::Type<TC1>()));
  EXPECT_NE(types.end(), std::find(types.begin(), types.end(),
        gazebo::ecs::ComponentFactory::Type<TC2>()));
  EXPECT_EQ(types.end(), std::find(types.begin(), types.end(),
        gazebo::ecs::ComponentFactory::Type<TC3>()));
}

/////////////////////////////////////////////////
TEST(EntityQuery, DuplicateComponent)
{
  gazebo::ecs::EntityQuery uut;
  uut.AddComponent("TC1");
  uut.AddComponent("TC1");
  
  std::set<gazebo::ecs::ComponentType> types = uut.ComponentTypes();
  ASSERT_EQ(1, types.size());
}

/////////////////////////////////////////////////
TEST(EntityQuery, ShallowCopyQuery)
{
  gazebo::ecs::EntityQuery uut;
  gazebo::ecs::EntityQuery uutCopy(uut);

  EXPECT_TRUE(uutCopy == uut);
}

/////////////////////////////////////////////////
TEST(EntityQuery, UnequalQueries)
{
  gazebo::ecs::EntityQuery uut;
  gazebo::ecs::EntityQuery uut2;

  EXPECT_FALSE(uut2 == uut);
}

/////////////////////////////////////////////////
TEST(EntityQuery, InitiallyNoResults)
{
  gazebo::ecs::EntityQuery uut;
  EXPECT_EQ(0, uut.EntityIds().size());
}


int main(int argc, char **argv)
{
  // Register types with the factory
  gazebo::ecs::ComponentFactory::Register<TC1>("TC1");
  gazebo::ecs::ComponentFactory::Register<TC2>("TC2");
  gazebo::ecs::ComponentFactory::Register<TC3>("TC3");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
