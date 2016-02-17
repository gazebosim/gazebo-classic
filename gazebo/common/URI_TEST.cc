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

#include "gazebo/common/Uri.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
TEST(URITEST, Scheme)
{
  URI uri;
  EXPECT_TRUE(uri.Str().empty());

  uri.SetScheme("data");
  EXPECT_EQ(uri.Str(), "data://");
}

/////////////////////////////////////////////////
TEST(URITEST, Path)
{
  URI uri;
  uri.SetScheme("data");

  uri.Path() = uri.Path() / "world";
  EXPECT_EQ(uri.Str(), "data://world");

  uri.Path() /= "default";
  EXPECT_EQ(uri.Str(), "data://world/default");
}

/////////////////////////////////////////////////
TEST(URITEST, Query)
{
  URI uri;
  uri.SetScheme("data");

  uri.Query().Add("p", "v");
  EXPECT_EQ(uri.Str(), "data://?p=v");

  uri.Path().PushFront("default");
  EXPECT_EQ(uri.Str(), "data://default?p=v");

  uri.Path().PushFront("world");
  EXPECT_EQ(uri.Str(), "data://world/default?p=v");

  URI uri2 = uri;

  uri.Path().Clear();
  EXPECT_EQ(uri.Str(), "data://?p=v");

  uri.Query().Clear();
  EXPECT_EQ(uri.Str(), "data://");

  uri.Clear();
  uri2.Clear();
  EXPECT_EQ(uri, uri2);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
