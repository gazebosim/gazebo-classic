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
using namespace common;

/////////////////////////////////////////////////
TEST(UriTest, UriEntityInvalid)
{
  EXPECT_THROW(UriEntity().SetName("_wrong id_"), Exception);
  EXPECT_THROW(UriEntity().SetName("_wrong?id_"), Exception);
  EXPECT_THROW(UriEntity().SetName("_wrong=id_"), Exception);
  EXPECT_THROW(UriEntity().SetName("_wrong&id_"), Exception);
  EXPECT_THROW(UriEntity().SetType("_wrong id_"), Exception);
  EXPECT_THROW(UriEntity().SetType("_wrong?id_"), Exception);
  EXPECT_THROW(UriEntity().SetType("_wrong=id_"), Exception);
  EXPECT_THROW(UriEntity().SetType("_wrong&id_"), Exception);

  EXPECT_THROW(UriEntity("_wrong id_", "name"), Exception);
  EXPECT_THROW(UriEntity("type", "_wrong id_"), Exception);
}

/////////////////////////////////////////////////
TEST(UriTest, UriEntity)
{
  // Accessors.
  UriEntity entity1, entity2;
  entity1.SetType("type1");
  EXPECT_EQ(entity1.Type(), "type1");
  entity1.SetName("name1");
  EXPECT_EQ(entity1.Name(), "name1");

  // Constructor.
  UriEntity entity3("type3", "name3");
  EXPECT_EQ(entity3.Type(), "type3");
  EXPECT_EQ(entity3.Name(), "name3");

  // This should make a copy.
  entity2 = entity1;

  // Modify entity1 to make sure that entity2 does not change.
  entity1.SetType("modified_type1");
  entity1.SetName("modified_name1");

  EXPECT_EQ(entity2.Type(), "type1");
  EXPECT_EQ(entity2.Name(), "name1");

  // Copy constructor;
  UriEntity entity4(entity2);
  EXPECT_EQ(entity4.Type(), "type1");
  EXPECT_EQ(entity4.Name(), "name1");
}

/////////////////////////////////////////////////
TEST(UriTest, UriNestedEntity)
{
  // Populate some entities.
  UriEntity entity, entity1, entity2, entity3;

  entity1.SetType("type1");
  entity1.SetName("name1");
  entity2.SetType("type2");
  entity2.SetName("name2");
  entity3.SetType("type3");
  entity3.SetName("name3");

  UriNestedEntity nestedEntity;
  EXPECT_EQ(nestedEntity.EntityCount(), 0u);

  // Trying to get an entity when nestedEntity is empty.
  EXPECT_THROW(nestedEntity.Parent(), Exception);
  EXPECT_THROW(nestedEntity.Leaf(), Exception);
  EXPECT_THROW(nestedEntity.Entity(0), Exception);

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

  EXPECT_THROW(nestedEntity.Entity(3), Exception);
  entity = nestedEntity.Entity(1);
  EXPECT_EQ(entity.Type(), "type2");
  EXPECT_EQ(entity.Name(), "name2");

  // This should create a copy.
  UriNestedEntity nestedEntity2 = nestedEntity;

  nestedEntity.Clear();
  EXPECT_EQ(nestedEntity.EntityCount(), 0u);

  entity = nestedEntity2.Parent();
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");
  entity = nestedEntity2.Leaf();
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");

  // Copy constructor.
  UriNestedEntity nestedEntity3(nestedEntity2);
  nestedEntity2.Clear();

  entity = nestedEntity3.Parent();
  EXPECT_EQ(entity.Type(), "type1");
  EXPECT_EQ(entity.Name(), "name1");
  entity = nestedEntity3.Leaf();
  EXPECT_EQ(entity.Type(), "type3");
  EXPECT_EQ(entity.Name(), "name3");
}

/////////////////////////////////////////////////
TEST(UriTest, UriPartsInvalid)
{
  // Empty.
  EXPECT_THROW(UriParts(""), Exception);
  EXPECT_THROW(UriParts().Parse(""), Exception);
  // Empty.
  EXPECT_THROW(UriParts("/"), Exception);
  EXPECT_THROW(UriParts().Parse("/"), Exception);
  // Invalid world keyword.
  EXPECT_THROW(UriParts("/wor/default/link/link1"), Exception);
  EXPECT_THROW(UriParts().Parse("/wor/default/link/link1"), Exception);
  // No world name.
  EXPECT_THROW(UriParts("/world"), Exception);
  EXPECT_THROW(UriParts().Parse("/world"), Exception);
  // Invalid world name.
  EXPECT_THROW(UriParts("/world/ default/link/link1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/ default/link/link1"), Exception);
  // Invalid world name.
  EXPECT_THROW(UriParts("/world/?default/link/link1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/?default/link/link1"), Exception);
  // Invalid world name.
  EXPECT_THROW(UriParts("/world/=default/link/link1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/=default/link/link1"), Exception);
  // Invalid world name.
  EXPECT_THROW(UriParts("/world/&default/link/link1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/&default/link/link1"), Exception);
  // No entity name.
  EXPECT_THROW(UriParts("/world/default/link"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link"), Exception);
  // No entity name.
  EXPECT_THROW(UriParts("/world/default/link/"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/"), Exception);
  // Invalid entity type.
  EXPECT_THROW(UriParts("/world/default/ link/link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/ link/link_1"), Exception);
  // Invalid entity type.
  EXPECT_THROW(UriParts("/world/default/?link/link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/?link/link_1"), Exception);
  // Invalid entity type.
  EXPECT_THROW(UriParts("/world/default/=link/link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/=link/link_1"), Exception);
  // Invalid entity type.
  EXPECT_THROW(UriParts("/world/default/&link/link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/&link/link_1"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/ link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/ link_1"), Exception);
  // empty entity name.
  EXPECT_THROW(UriParts("/world/default/link/?link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/?link_1"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/=link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/=link_1"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/&link_1"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/&link_1"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/l1/link/ l2"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1/link/ l2"), Exception);
  // empty entity name.
  EXPECT_THROW(UriParts("/world/default/link/l1/link/?l2"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1/link/?l2"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/l1/link/=l2"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1/link/=l2"), Exception);
  // Invalid entity name.
  EXPECT_THROW(UriParts("/world/default/link/l1/link/&l2"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1/link/&l2"), Exception);
  // Invalid parameter.
  EXPECT_THROW(UriParts("/world/default/link/l1=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1=pose"), Exception);
  // Invalid parameter (extra '/').
  EXPECT_THROW(UriParts("/world/default/link/l1/?p=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1/?p=pose"), Exception);
  // Invalid parameter (no '=').
  EXPECT_THROW(UriParts("/world/default/link/l1?p"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p"), Exception);
  // Invalid parameter (no value).
  EXPECT_THROW(UriParts("/world/default/link/l1?p="), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p="), Exception);
  // Invalid parameter name.
  EXPECT_THROW(UriParts("/world/default/link/l1??p=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1??p=pose"), Exception);
  // Invalid parameter name.
  EXPECT_THROW(UriParts("/world/default/link/l1? p=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1? p=pose"), Exception);
  // Invalid parameter name.
  EXPECT_THROW(UriParts("/world/default/link/l1?=p=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?=p=pose"), Exception);
  // Invalid parameter name.
  EXPECT_THROW(UriParts("/world/default/link/l1?&p=pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?&p=pose"), Exception);
  // Invalid parameter value.
  EXPECT_THROW(UriParts("/world/default/link/l1?p= pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p= pose"), Exception);
  // Invalid parameter value.
  EXPECT_THROW(UriParts("/world/default/link/l1?p=?pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p=?pose"), Exception);
  // Invalid parameter value.
  EXPECT_THROW(UriParts("/world/default/link/l1?p==pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p==pose"), Exception);
  // Invalid parameter value.
  EXPECT_THROW(UriParts("/world/default/link/l1?p=&pose"), Exception);
  EXPECT_THROW(UriParts().Parse("/world/default/link/l1?p=&pose"), Exception);
}

/////////////////////////////////////////////////
TEST(UriTest, UriPartsStringConstructors)
{
  // Constructors and Parse() using a URI string.
  EXPECT_NO_THROW(UriParts("/world/default"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default"));
  EXPECT_NO_THROW(UriParts("/world/default/"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/"));
  EXPECT_NO_THROW(UriParts("/world/default?p=time"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default?p=time"));
  EXPECT_NO_THROW(UriParts("/world/default/link/link1"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/link1"));
  EXPECT_NO_THROW(UriParts("/world/default/link/l1/link/l2"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/l1/link/l2"));
  EXPECT_NO_THROW(UriParts("/world/default/link/link1?p=pose"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/link1?p=pose"));
  EXPECT_NO_THROW(UriParts("/world/default/link/l1/link/l2?p=pose"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/l1/link/l2?p=pose"));
  EXPECT_NO_THROW(UriParts("/world/default/link/link1/"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/link1/"));
  EXPECT_NO_THROW(UriParts("/world/default/link/l1/link/l2/"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/l1/link/l2/"));
  EXPECT_NO_THROW(UriParts("/world/default/link/link1?p=pose/"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/link1?p=pose/"));
  EXPECT_NO_THROW(UriParts("/world/default/link/l1/link/l2?p=pose/"));
  EXPECT_NO_THROW(UriParts().Parse("/world/default/link/l1/link/l2?p=pose/"));
}

/////////////////////////////////////////////////
TEST(UriTest, UriParts)
{
  // Populate some URI entities.
  UriEntity entity, entity1, entity2, entity3;

  entity1.SetType("type1");
  entity1.SetName("name1");
  entity2.SetType("type2");
  entity2.SetName("name2");
  entity3.SetType("type3");
  entity3.SetName("name3");

  // Add the entities to a NestedEntity.
  UriNestedEntity nestedEntity;
  nestedEntity.AddEntity(entity1);
  nestedEntity.AddEntity(entity2);
  nestedEntity.AddEntity(entity3);

  UriParts uriParts;

  uriParts.SetWorld("world1");
  uriParts.SetEntity(nestedEntity);
  uriParts.SetParameter("param1");

  EXPECT_EQ(uriParts.World(), "world1");
  auto nestedEntity2 = uriParts.Entity();
  EXPECT_EQ(nestedEntity2.Parent().Type(), "type1");
  EXPECT_EQ(nestedEntity2.Parent().Name(), "name1");
  EXPECT_EQ(nestedEntity2.Leaf().Type(), "type3");
  EXPECT_EQ(nestedEntity2.Leaf().Name(), "name3");
  auto p = uriParts.Parameter();
  EXPECT_EQ(p, "param1");

  // Copy constructor.
  UriParts uriParts2(uriParts);

  // Modify uriparPs to make sure that uriParts2 does not change.
  uriParts.SetWorld("modified_world1");
  uriParts.Entity().Clear();
  uriParts.SetParameter("");

  EXPECT_EQ(uriParts2.World(), "world1");
  nestedEntity = uriParts2.Entity();
  EXPECT_EQ(nestedEntity.Parent().Type(), "type1");
  EXPECT_EQ(nestedEntity.Parent().Name(), "name1");
  EXPECT_EQ(nestedEntity.Leaf().Type(), "type3");
  EXPECT_EQ(nestedEntity.Leaf().Name(), "name3");
  p = uriParts2.Parameter();
  EXPECT_EQ(p, "param1");

  // Assignment operator.
  EXPECT_EQ(uriParts.Entity().EntityCount(), 0u);
  EXPECT_TRUE(uriParts.Parameter().empty());

  uriParts = uriParts2;

  // Modify uriParts2 to make sure that uriParts does not change.
  uriParts2.SetWorld("modified_world1");
  uriParts2.Entity().Clear();
  uriParts2.SetParameter("");

  EXPECT_EQ(uriParts.World(), "world1");
  nestedEntity = uriParts.Entity();
  EXPECT_EQ(nestedEntity.Parent().Type(), "type1");
  EXPECT_EQ(nestedEntity.Parent().Name(), "name1");
  EXPECT_EQ(nestedEntity.Leaf().Type(), "type3");
  EXPECT_EQ(nestedEntity.Leaf().Name(), "name3");
  p = uriParts.Parameter();
  EXPECT_EQ(p, "param1");
}

/////////////////////////////////////////////////
TEST(UriTest, UriInvalid)
{
  // Missing /world
  EXPECT_THROW(Uri("/def/model/model_1"), Exception);
  // Missing /world
  EXPECT_THROW(Uri("/incorrect/def/model/model_1/"), Exception);
  // Missing entity name
  EXPECT_THROW(Uri("/world/def/model"), Exception);
  // Missing entity name
  EXPECT_THROW(Uri("/world/def/model/"), Exception);
  // Wrong entity name (white space)
  EXPECT_THROW(Uri("/world/def/model/ "), Exception);
  // Invalid '/' before the parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1/p=pose"), Exception);
  // Missing ? in parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1p=pose"), Exception);
  // Invalid '/' before the parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1/?pose"), Exception);
  // Missing = in parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1?pose"), Exception);
  // Invalid '/' before the parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1/?p="), Exception);
  // Missing right argument after "=" in parameter list.
  EXPECT_THROW(Uri("/world/def/model/model_1?p="), Exception);
  // Invalid entity name (contains '=').
  EXPECT_THROW(Uri("/world/def/model/model_1p=pose"), Exception);
  // Missing part of the parameter.
  EXPECT_THROW(Uri("/world/def/model/model_1p&pose"), Exception);
  // Invalid entity name (contains '=').
  EXPECT_THROW(Uri("/world/def/model/model_1p?pose"), Exception);
  // Invalid parameter name (contains '&').
  EXPECT_THROW(Uri("/world/def/model/model_1?p=pose&p=lin_vel"), Exception);
}

/////////////////////////////////////////////////
TEST(UriTest, UriStringConstructor)
{
  Uri uri1("/world/def/model/model_1");
  auto parts = uri1.Parts();
  EXPECT_EQ(parts.World(), "def");
  UriEntity entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri1.CanonicalUri(), "/world/def/model/model_1");
  EXPECT_EQ(uri1.CanonicalUri("pose"), "/world/def/model/model_1?p=pose");

  Uri uri2("/world/def/model/model_1/");
  parts = uri2.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri2.CanonicalUri(), "/world/def/model/model_1");
  EXPECT_EQ(uri2.CanonicalUri("pose"), "/world/def/model/model_1?p=pose");

  Uri uri3("/world/def/model/model_1/model/model_2");
  parts = uri3.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri3.CanonicalUri(), "/world/def/model/model_1/model/model_2");
  EXPECT_EQ(uri3.CanonicalUri("pose"),
      "/world/def/model/model_1/model/model_2?p=pose");

  Uri uri4("/world/def/model/model_1/model/model_2/");
  parts = uri4.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri4.CanonicalUri(), "/world/def/model/model_1/model/model_2");
  EXPECT_EQ(uri4.CanonicalUri("pose"),
      "/world/def/model/model_1/model/model_2?p=pose");

  Uri uri5("/world/def/model/model_1?p=pose");
  parts = uri5.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_EQ(uri5.CanonicalUri(), "/world/def/model/model_1?p=pose");
  EXPECT_EQ(uri5.CanonicalUri("lin_vel"),
      "/world/def/model/model_1?p=lin_vel");

  Uri uri6("/world/def/model/model_1/model/model_2?p=pose/pos/x");
  parts = uri6.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  entity = parts.Entity().Leaf();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_2");
  EXPECT_EQ(uri6.CanonicalUri(),
      "/world/def/model/model_1/model/model_2?p=pose/pos/x");
  EXPECT_EQ(uri6.CanonicalUri("lin_vel"),
      "/world/def/model/model_1/model/model_2?p=lin_vel");

  Uri uri7("/world/def/model/model_1");
  parts = uri7.Parts();
  EXPECT_EQ(parts.World(), "def");
  entity = parts.Entity().Parent();
  EXPECT_EQ(entity.Type(), "model");
  EXPECT_EQ(entity.Name(), "model_1");
  EXPECT_TRUE(parts.Parameter().empty());
  EXPECT_EQ(uri7.CanonicalUri("pose"), "/world/def/model/model_1?p=pose");
  EXPECT_EQ(uri7.CanonicalUri("lin_vel"), "/world/def/model/model_1?p=lin_vel");

  // Invalid parameters in CanonicalUri().
  EXPECT_THROW(uri7.CanonicalUri(" pose"), Exception);
  EXPECT_THROW(uri7.CanonicalUri("?pose"), Exception);
  EXPECT_THROW(uri7.CanonicalUri("=pose"), Exception);
  EXPECT_THROW(uri7.CanonicalUri("pose&"), Exception);
  EXPECT_THROW(uri7.CanonicalUri("?p=pose"), Exception);
}

/////////////////////////////////////////////////
TEST(UriTest, UriOtherConstructors)
{
  Uri uri1("/world/def/model/model_1?p=pose");
  auto parts = uri1.Parts();
  Uri uri2(parts);

  // Modify uri1 to make sure uri2 is a separate copy.
  uri1.Parts().SetWorld("modified_world");

  EXPECT_EQ(uri2.Parts().World(), "def");
  EXPECT_EQ(uri2.Parts().Entity().Parent().Name(), "model_1");
  EXPECT_EQ(uri2.Parts().Parameter(), "pose");

  // Copy constructor.
  Uri uri3(uri2);

  // Modify uri2 to make sure uri3 is a separate copy.
  uri2.Parts().SetWorld("modified_world");

  EXPECT_EQ(uri3.Parts().World(), "def");
  EXPECT_EQ(uri3.Parts().Entity().Parent().Name(), "model_1");
  EXPECT_EQ(uri3.Parts().Parameter(), "pose");
}
