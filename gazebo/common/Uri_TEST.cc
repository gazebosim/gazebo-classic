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

#include <gtest/gtest.h>

#include "gazebo/common/Uri.hh"

using namespace gazebo;

/////////////////////////////////////////////////
TEST(UriTest, UriEntity)
{
  common::UriEntity entity1, entity2;
  entity1.SetType("type1");
  EXPECT_EQ(entity1.Type(), "type1");
  entity1.SetName("name1");
  EXPECT_EQ(entity1.Name(), "name1");

  // This should make a copy.
  entity2 = entity1;

  // Modify entity1 to make sure that entity2 does not change.
  entity1.SetType("modified_type1");
  entity1.SetName("modified_name1");

  EXPECT_EQ(entity2.Type(), "type1");
  EXPECT_EQ(entity2.Name(), "name1");

  // Copy constructor;
  common::UriEntity entity3(entity2);
  EXPECT_EQ(entity3.Type(), "type1");
  EXPECT_EQ(entity3.Name(), "name1");
}


/////////////////////////////////////////////////
TEST(UriTest, UriNestedEntity)
{
  // Populate some entities.
  common::UriEntity entity, entity1, entity2, entity3;

  entity1.SetType("type1");
  entity1.SetName("name1");
  entity2.SetType("type2");
  entity2.SetName("name2");
  entity3.SetType("type3");
  entity3.SetName("name3");

  // Add the entities to a NestedEntity.
  common::UriNestedEntity nestedEntity;
  nestedEntity.AddEntity(entity1);
  nestedEntity.AddEntity(entity2);
  nestedEntity.AddEntity(entity3);

  EXPECT_TRUE(nestedEntity.Parent(entity));
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");

  EXPECT_TRUE(nestedEntity.Leaf(entity));
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");

  EXPECT_EQ(nestedEntity.EntityCount(), 3u);

  EXPECT_FALSE(nestedEntity.Entity(3, entity));
  EXPECT_TRUE(nestedEntity.Entity(1, entity));
  EXPECT_EQ(entity.Type(), "type2");
  EXPECT_EQ(entity.Name(), "name2");
}

/////////////////////////////////////////////////
TEST(UriTest, UriParts)
{
  // Populate some entities.
  common::UriEntity entity, entity1, entity2, entity3;

  entity1.SetType("type1");
  entity1.SetName("name1");
  entity2.SetType("type2");
  entity2.SetName("name2");
  entity3.SetType("type3");
  entity3.SetName("name3");

  // Add the entities to a NestedEntity.
  common::UriNestedEntity nestedEntity;
  nestedEntity.AddEntity(entity1);
  nestedEntity.AddEntity(entity2);
  nestedEntity.AddEntity(entity3);

  common::UriParts uriParts;

  uriParts.SetWorld("world1");
  uriParts.SetEntity(nestedEntity);
  uriParts.SetParameters({"param1"});
}

/*
/////////////////////////////////////////////////
TEST(UriTest, UriEntityPart)
{
  common::UriEntityPart entity, entity2;
  entity.SetType("parent_type");
  EXPECT_EQ(entity.Type(), "parent_type");
  entity.SetName("parent_name");
  EXPECT_EQ(entity.Name(), "parent_name");
  EXPECT_TRUE(entity.Children() == NULL);

  // Add a child.
  auto child1 = std::make_shared<common::UriEntityPart>();
  child1->SetType("type1");
  child1->SetName("name1");
  entity.SetChildren(child1);

  ASSERT_TRUE(entity.Children() != NULL);
  EXPECT_EQ(entity.Children()->Type(), "type1");
  EXPECT_EQ(entity.Children()->Name(), "name1");

  // Add a second child.
  auto child2 = std::make_shared<common::UriEntityPart>();
  child2->SetType("type2");
  child2->SetName("name2");
  auto child = entity.Children();
  child->SetChildren(child2);

  ASSERT_TRUE(entity.Children()->Children() != NULL);
  EXPECT_EQ(entity.Children()->Children()->Type(), "type2");
  EXPECT_EQ(entity.Children()->Children()->Name(), "name2");

  // Should create a copy.
  entity2 = entity;

  // Modify entity and make sure that entity2 does not change.
  entity.SetName("parant_name_modified");
  entity.SetType("parant_type_modified");
  entity.Children()->SetName("name1_modified");
  entity.Children()->SetType("type1_modified");
  entity.Children()->Children()->SetName("name2_modified");
  entity.Children()->Children()->SetType("type2_modified");

  EXPECT_EQ(entity2.Name(), "parent_name");
  EXPECT_EQ(entity2.Type(), "parent_type");
  ASSERT_TRUE(entity2.Children() != NULL);
  EXPECT_EQ(entity2.Children()->Name(), "name1");
  EXPECT_EQ(entity2.Children()->Type(), "type1");
  ASSERT_TRUE(entity2.Children()->Children() != NULL);
  EXPECT_EQ(entity2.Children()->Children()->Name(), "name2");
  EXPECT_EQ(entity2.Children()->Children()->Type(), "type2");
}

/////////////////////////////////////////////////
//TEST(UriTest, UriParts)
//{
//
//}

*/

/////////////////////////////////////////////////
TEST(UriTest, Parse)
{
  common::Uri uri;
  common::UriParts parts;
  common::UriEntity entity;

  EXPECT_FALSE(uri.Parse("/def/model/model_1", parts));
  EXPECT_FALSE(uri.Parse("/incorrect/def/model/model_1/", parts));

  EXPECT_FALSE(uri.Parse("/world/def/model", parts));
  EXPECT_FALSE(uri.Parse("/world/def/model/", parts));
  EXPECT_FALSE(uri.Parse("/world/def/model/ ", parts));

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1", parts));
  EXPECT_EQ(parts.World(), "def");
  EXPECT_TRUE(parts.Entity().Parent(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/", parts));
  EXPECT_EQ(parts.World(), "def");
  EXPECT_TRUE(parts.Entity().Parent(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/model/model_2", parts));
  EXPECT_EQ(parts.World(), "def");
  EXPECT_TRUE(parts.Entity().Parent(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_TRUE(parts.Entity().Leaf(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/model/model_2/", parts));
  EXPECT_EQ(parts.World(), "def");
  EXPECT_TRUE(parts.Entity().Parent(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_TRUE(parts.Entity().Leaf(entity));
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
}
