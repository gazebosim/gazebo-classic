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

#include <algorithm>
#include <gtest/gtest.h>

#include "gazebo/common/Exception.hh"
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

  // Invalid identifiers.
  EXPECT_THROW(entity3.SetName("_wrong identifier_"), common::Exception);
  EXPECT_THROW(entity3.SetName("_wrong?identifier_"), common::Exception);
  EXPECT_THROW(entity3.SetType("_wrong identifier_"), common::Exception);
  EXPECT_THROW(entity3.SetType("_wrong?identifier_"), common::Exception);
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

  common::UriNestedEntity nestedEntity;
  EXPECT_EQ(nestedEntity.EntityCount(), 0u);

  // Trying to get an entity when nestedEntity is empty.
  EXPECT_THROW(nestedEntity.Parent(), common::Exception);
  EXPECT_THROW(nestedEntity.Leaf(), common::Exception);
  EXPECT_THROW(nestedEntity.Entity(0), common::Exception);

  // Add the entities to a NestedEntity.
  nestedEntity.AddEntity(entity1);
  nestedEntity.AddEntity(entity2);
  nestedEntity.AddEntity(entity3);

  entity = nestedEntity.Parent();
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");

  entity = nestedEntity.Leaf();
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");

  EXPECT_EQ(nestedEntity.EntityCount(), 3u);

  EXPECT_THROW(nestedEntity.Entity(3), common::Exception);
  entity = nestedEntity.Entity(1);
  EXPECT_EQ(entity.Type(), "type2");
  EXPECT_EQ(entity.Name(), "name2");

  // This should create a copy.
  common::UriNestedEntity nestedEntity2 = nestedEntity;

  nestedEntity.Clear();
  EXPECT_EQ(nestedEntity.EntityCount(), 0u);

  entity = nestedEntity2.Parent();
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");
  entity = nestedEntity2.Leaf();
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");

  // Copy constructor.
  common::UriNestedEntity nestedEntity3(nestedEntity2);
  nestedEntity2.Clear();

  entity = nestedEntity3.Parent();
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");
  entity = nestedEntity3.Leaf();
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");
}

/////////////////////////////////////////////////
TEST(UriTest, UriParts)
{
  // Populate some URI entities.
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
  uriParts.SetParameters({"param1", "param2"});

  EXPECT_EQ(uriParts.World(), "world1");
  auto nestedEntity2 = uriParts.Entity();
  EXPECT_EQ(nestedEntity2.Parent().Type(), "type1");
  EXPECT_EQ(nestedEntity2.Parent().Name(), "name1");
  EXPECT_EQ(nestedEntity2.Leaf().Type(), "type3");
  EXPECT_EQ(nestedEntity2.Leaf().Name(), "name3");
  auto p = uriParts.Parameters();
  EXPECT_EQ(p.size(), 2u);
  EXPECT_TRUE(std::find(p.begin(), p.end(), "param1") != p.end());
  EXPECT_TRUE(std::find(p.begin(), p.end(), "param2") != p.end());
}

/////////////////////////////////////////////////
TEST(UriTest, Parse)
{
  common::UriEntity entity;

  // Missing /world
  EXPECT_THROW(common::Uri("/def/model/model_1"), common::Exception);
  // Missing /world
  EXPECT_THROW(common::Uri("/incorrect/def/model/model_1/"), common::Exception);

  // Missing entity name
  EXPECT_THROW(common::Uri("/world/def/model"), common::Exception);
  // Missing entity name
  EXPECT_THROW(common::Uri("/world/def/model/"), common::Exception);
  // Wrong entity name (white space)
  EXPECT_THROW(common::Uri("/world/def/model/ "), common::Exception);

  // Missing ? in parameter list.
  EXPECT_THROW(common::Uri("/world/def/model/model_1/p=pose"),
      common::Exception);

  // Missing = in parameter list.
  EXPECT_THROW(common::Uri("/world/def/model/model_1/?pose"),
      common::Exception);

  // Missing right argument after "=" in parameter list.
  EXPECT_THROW(common::Uri("/world/def/model/model_1/?p="),
      common::Exception);

  common::Uri uri1("/world/def/model/model_1");
  auto parts = uri1.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri1.CanonicalUri(), "/world/def/model/model_1");

  common::Uri uri2("/world/def/model/model_1/");
  parts = uri2.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri2.CanonicalUri(), "/world/def/model/model_1");

  common::Uri uri3("/world/def/model/model_1/model/model_2");
  parts = uri3.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri3.CanonicalUri(), "/world/def/model/model_1/model/model_2");

  common::Uri uri4("/world/def/model/model_1/model/model_2/");
  parts = uri4.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri4.CanonicalUri(), "/world/def/model/model_1/model/model_2");

  common::Uri uri5("/world/def/model/model_1?p=pose");
  parts = uri5.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri5.CanonicalUri(), "/world/def/model/model_1?p=pose");

  common::Uri uri6("/world/def/model/model_1?p=pose&p=lin_vel");
  parts = uri6.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri6.CanonicalUri(), "/world/def/model/model_1?p=pose&p=lin_vel");

  common::Uri uri7("/world/def/model/model_1/model/model_2?p=pose/pos/x");
  parts = uri7.Split();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri7.CanonicalUri(),
    "/world/def/model/model_1/model/model_2?p=pose/pos/x");
}
