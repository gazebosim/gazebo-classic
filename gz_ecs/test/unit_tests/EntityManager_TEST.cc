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

#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/ecs_core/EntityQuery.hh"
#include "gazebo/ecs_core/EntityQueryResult.hh"

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
TEST(EntityManager, FirstEntityIsNotNoEntity)
{
  gazebo::ecs_core::EntityManager uut;
  gazebo::ecs_core::Entity entity;
  entity = uut.CreateEntity();
  EXPECT_NE(gazebo::ecs_core::NO_ENTITY, entity);
}

/////////////////////////////////////////////////
TEST(EntityManager, CreateUniqueEntities)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;

  for (int i = 0; i < 100; i++)
  {
    gazebo::ecs_core::Entity e = uut.CreateEntity();
    EXPECT_EQ(entities.end(), std::find(entities.begin(), entities.end(), e));
    entities.push_back(e);
  }
}

/////////////////////////////////////////////////
TEST(EntityManager, AddComponentToOneEntity)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[2]);

  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[1]));
  EXPECT_NE(nullptr, uut.GetComponent<TC1>(entities[2]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityManager, AddDifferentComponentsToDifferentEntities)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);

  EXPECT_NE(nullptr, uut.GetComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[1]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[2]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[3]));
  
  EXPECT_EQ(nullptr, uut.GetComponent<TC2>(entities[0]));
  EXPECT_NE(nullptr, uut.GetComponent<TC2>(entities[1]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC2>(entities[2]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC2>(entities[3]));
  
  EXPECT_EQ(nullptr, uut.GetComponent<TC3>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC3>(entities[1]));
  EXPECT_NE(nullptr, uut.GetComponent<TC3>(entities[2]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC3>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityManager, AddMultipleComponentsToOneEntity)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC3>(entities[0]);

  EXPECT_NE(nullptr, uut.GetComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[1]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC1>(entities[2]));
  
  EXPECT_NE(nullptr, uut.GetComponent<TC2>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC2>(entities[1]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC2>(entities[2]));
  
  EXPECT_NE(nullptr, uut.GetComponent<TC3>(entities[0]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC3>(entities[1]));
  EXPECT_EQ(nullptr, uut.GetComponent<TC3>(entities[2]));
}

/////////////////////////////////////////////////
TEST(EntityManager, ComponentsAreInitiallyEditable)
{
  gazebo::ecs_core::EntityManager uut;
  gazebo::ecs_core::Entity entity;
  entity = uut.CreateEntity();

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;
  
  auto first = uut.AddComponent<TC1>(entity);
  first->itemOne = one_one;
  auto second = uut.AddComponent<TC2>(entity);
  second->itemOne = two_one;
  second->itemTwo = two_two;
  auto third = uut.AddComponent<TC3>(entity);
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;

  ASSERT_NE(nullptr, uut.GetComponent<TC1>(entity));
  ASSERT_NE(nullptr, uut.GetComponent<TC2>(entity));
  ASSERT_NE(nullptr, uut.GetComponent<TC3>(entity));
  
  EXPECT_EQ(one_one, uut.GetComponent<TC1>(entity)->itemOne);
  EXPECT_EQ(two_one, uut.GetComponent<TC2>(entity)->itemOne);
  EXPECT_EQ(two_two, uut.GetComponent<TC2>(entity)->itemTwo);
  EXPECT_EQ(three_one, uut.GetComponent<TC3>(entity)->itemOne);
  EXPECT_EQ(three_two, uut.GetComponent<TC3>(entity)->itemTwo);
  EXPECT_EQ(three_three, uut.GetComponent<TC3>(entity)->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityManager, ComponentsAreAlwaysEditable)
{
  gazebo::ecs_core::EntityManager uut;
  gazebo::ecs_core::Entity entity;
  entity = uut.CreateEntity();
  uut.AddComponent<TC1>(entity);
  uut.AddComponent<TC2>(entity);
  uut.AddComponent<TC3>(entity);

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;
  
  auto first = uut.GetComponent<TC1>(entity);
  auto second = uut.GetComponent<TC2>(entity);
  auto third = uut.GetComponent<TC3>(entity);
  ASSERT_NE(nullptr, first);
  ASSERT_NE(nullptr, second);
  ASSERT_NE(nullptr, third);

  first->itemOne = one_one;
  second->itemOne = two_one;
  second->itemTwo = two_two;
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;
  
  EXPECT_EQ(one_one, uut.GetComponent<TC1>(entity)->itemOne);
  EXPECT_EQ(two_one, uut.GetComponent<TC2>(entity)->itemOne);
  EXPECT_EQ(two_two, uut.GetComponent<TC2>(entity)->itemTwo);
  EXPECT_EQ(three_one, uut.GetComponent<TC3>(entity)->itemOne);
  EXPECT_EQ(three_two, uut.GetComponent<TC3>(entity)->itemTwo);
  EXPECT_EQ(three_three, uut.GetComponent<TC3>(entity)->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityManager, QueryEntities)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[0]);
  uut.AddComponent<TC3>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);

  gazebo::ecs_core::EntityQuery query;
  query.AddComponent<TC2>();
  query.AddComponent<TC3>();

  uut.AddQuery(query);
  uut.Update();

  ASSERT_EQ(2, query.Results()->NumResults());
  bool foundE1 = false;
  bool foundE2 = false;
  bool foundE3 = false;
  for (int i = 0; i < 2; i++)
  {
    gazebo::ecs_core::Entity entity = query.Results()->At(i);
    if (entity == entities[0])
      foundE1 = true;
    else if (entity == entities[1])
      foundE2 = true;
    else
      foundE3 = true;
  }

  EXPECT_TRUE(foundE1);
  EXPECT_TRUE(foundE2);
  EXPECT_FALSE(foundE3);
}

/////////////////////////////////////////////////
TEST(EntityManager, RemoveQueryNoResults)
{
  gazebo::ecs_core::EntityManager uut;
  std::vector<gazebo::ecs_core::Entity> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[0]);
  uut.AddComponent<TC3>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);

  gazebo::ecs_core::EntityQuery query;
  query.AddComponent<TC2>();
  query.AddComponent<TC3>();

  uut.AddQuery(query);
  uut.RemoveQuery(query);
  uut.Update();

  ASSERT_EQ(0, query.Results()->NumResults());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
