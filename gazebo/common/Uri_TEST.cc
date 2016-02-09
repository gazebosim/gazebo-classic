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
TEST(UriTest, Parse)
{
  common::Uri uri;
  common::UriParts parts;

  EXPECT_FALSE(uri.Parse("/def/model/model_1", parts));
  EXPECT_FALSE(uri.Parse("/incorrect/def/model/model_1/", parts));

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1", parts));
  EXPECT_EQ(parts.world, "def");
  EXPECT_EQ(parts.entity.type, "model");
  EXPECT_EQ(parts.entity.name, "model_1");
  EXPECT_TRUE(parts.entity.children == NULL);

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/", parts));
  EXPECT_EQ(parts.world, "def");
  EXPECT_EQ(parts.entity.type, "model");
  EXPECT_EQ(parts.entity.name, "model_1");
  EXPECT_TRUE(parts.entity.children == NULL);

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/model/model_2", parts));
  EXPECT_EQ(parts.world, "def");
  EXPECT_EQ(parts.entity.type, "model");
  EXPECT_EQ(parts.entity.name, "model_1");
  EXPECT_TRUE(parts.entity.children != NULL);
  EXPECT_EQ(parts.entity.children->type, "model");
  EXPECT_EQ(parts.entity.children->name, "model_2");
  EXPECT_TRUE(parts.entity.children->children == NULL);

  EXPECT_TRUE(uri.Parse("/world/def/model/model_1/model/model_2/", parts));
  EXPECT_EQ(parts.world, "def");
  EXPECT_EQ(parts.entity.type, "model");
  EXPECT_EQ(parts.entity.name, "model_1");
  EXPECT_TRUE(parts.entity.children != NULL);
  EXPECT_EQ(parts.entity.children->type, "model");
  EXPECT_EQ(parts.entity.children->name, "model_2");
  EXPECT_TRUE(parts.entity.children->children == NULL);
}
