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
#include "gazebo/ecs/EntityComponentDatabase.hh"
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
TEST(EntityComponentDatabase, FirstEntityIdIsNotNoEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();
  EXPECT_NE(gazebo::ecs::NO_ENTITY, entity);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, CreateUniqueEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;

  for (int i = 0; i < 100; i++)
  {
    gazebo::ecs::EntityId e = uut.CreateEntity();
    EXPECT_EQ(entities.end(), std::find(entities.begin(), entities.end(), e));
    entities.push_back(e);
  }
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddComponentToOneEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type = gazebo::ecs::ComponentFactory::Type<TC1>();
  uut.AddComponent(entities[2], type);

  EXPECT_EQ(nullptr, uut.EntityComponent(entities[0], type));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type));
  EXPECT_NE(nullptr, uut.EntityComponent(entities[2], type));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[3], type));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddDifferentComponentsToDifferentEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  uut.AddComponent(entities[0], type1);
  uut.AddComponent(entities[1], type2);
  uut.AddComponent(entities[2], type3);

  EXPECT_NE(nullptr, uut.EntityComponent(entities[0], type1));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type1));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[2], type1));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[3], type1));
  
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[0], type2));
  EXPECT_NE(nullptr, uut.EntityComponent(entities[1], type2));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[2], type2));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[3], type2));
  
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[0], type3));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type3));
  EXPECT_NE(nullptr, uut.EntityComponent(entities[2], type3));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[3], type3));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddMultipleComponentsToOneEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  uut.AddComponent(entities[0], type1);
  uut.AddComponent(entities[0], type2);
  uut.AddComponent(entities[0], type3);

  EXPECT_NE(nullptr, uut.EntityComponent(entities[0], type1));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type1));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[2], type1));
  
  EXPECT_NE(nullptr, uut.EntityComponent(entities[0], type2));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type2));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[2], type2));
  
  EXPECT_NE(nullptr, uut.EntityComponent(entities[0], type3));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[1], type3));
  EXPECT_EQ(nullptr, uut.EntityComponent(entities[2], type3));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreInitiallyEditable)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  auto first = static_cast<TC1*>(uut.AddComponent(entity, type1));
  first->itemOne = one_one;
  auto second = static_cast<TC2*>(uut.AddComponent(entity, type2));
  second->itemOne = two_one;
  second->itemTwo = two_two;
  auto third = static_cast<TC3*>(uut.AddComponent(entity, type3));
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;

  ASSERT_NE(nullptr, uut.EntityComponent(entity, type1));
  ASSERT_NE(nullptr, uut.EntityComponent(entity, type2));
  ASSERT_NE(nullptr, uut.EntityComponent(entity, type3));
  
  EXPECT_EQ(one_one, static_cast<TC1*>(uut.EntityComponent(entity, type1))->itemOne);
  EXPECT_EQ(two_one, static_cast<TC2*>(uut.EntityComponent(entity, type2))->itemOne);
  EXPECT_EQ(two_two, static_cast<TC2*>(uut.EntityComponent(entity, type2))->itemTwo);
  EXPECT_EQ(three_one, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemOne);
  EXPECT_EQ(three_two, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemTwo);
  EXPECT_EQ(three_three, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreAlwaysEditable)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();
  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();
  uut.AddComponent(entity, type1);
  uut.AddComponent(entity, type2);
  uut.AddComponent(entity, type3);

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;

  auto first = static_cast<TC1*>(uut.EntityComponent(entity, type1));
  auto second = static_cast<TC2*>(uut.EntityComponent(entity, type2));
  auto third = static_cast<TC3*>(uut.EntityComponent(entity, type3));

  ASSERT_NE(nullptr, first);
  ASSERT_NE(nullptr, second);
  ASSERT_NE(nullptr, third);

  first->itemOne = one_one;
  second->itemOne = two_one;
  second->itemTwo = two_two;
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;


  EXPECT_EQ(one_one, static_cast<TC1*>(uut.EntityComponent(entity, type1))->itemOne);
  EXPECT_EQ(two_one, static_cast<TC2*>(uut.EntityComponent(entity, type2))->itemOne);
  EXPECT_EQ(two_two, static_cast<TC2*>(uut.EntityComponent(entity, type2))->itemTwo);
  EXPECT_EQ(three_one, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemOne);
  EXPECT_EQ(three_two, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemTwo);
  EXPECT_EQ(three_three, static_cast<TC3*>(uut.EntityComponent(entity, type3))->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueryEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  uut.AddComponent(entities[0], type1);
  uut.AddComponent(entities[0], type2);
  uut.AddComponent(entities[1], type2);
  uut.AddComponent(entities[0], type3);
  uut.AddComponent(entities[1], type3);
  uut.AddComponent(entities[2], type3);

  gazebo::ecs::EntityQuery query;
  query.AddComponent(type2);
  query.AddComponent(type3);

  uut.AddQuery(query);

  ASSERT_EQ(2, query.EntityIds().size());
  bool foundE1 = false;
  bool foundE2 = false;
  bool foundE3 = false;
  for (auto result : query.EntityIds())
  {
    if (result == entities[0])
      foundE1 = true;
    else if (result == entities[1])
      foundE2 = true;
    else
      foundE3 = true;
  }

  EXPECT_TRUE(foundE1);
  EXPECT_TRUE(foundE2);
  EXPECT_FALSE(foundE3);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveQueryNoResults)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  uut.AddComponent(entities[0], type1);
  uut.AddComponent(entities[0], type2);
  uut.AddComponent(entities[1], type2);
  uut.AddComponent(entities[0], type3);
  uut.AddComponent(entities[1], type3);
  uut.AddComponent(entities[2], type3);

  gazebo::ecs::EntityQuery query;
  query.AddComponent(type2);
  query.AddComponent(type3);

  uut.AddQuery(query);
  uut.RemoveQuery(query);

  ASSERT_EQ(0, query.EntityIds().size());
}

int main(int argc, char **argv)
{
  gazebo::ecs::ComponentFactory::Register<TC1>("TC1");
  gazebo::ecs::ComponentFactory::Register<TC2>("TC2");
  gazebo::ecs::ComponentFactory::Register<TC3>("TC3");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
