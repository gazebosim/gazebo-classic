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
#include "gazebo/common/Exception.hh"
#include "gazebo/common/URI.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
TEST(URITEST, URIPath)
{
  URIPath path1, path2, path3;
  EXPECT_TRUE(path1.Str().empty());

  path1.PushFront("part1");
  EXPECT_EQ(path1.Str(), "part1");
  path1.PushBack("part2");
  EXPECT_EQ(path1.Str(), "part1/part2");
  path1.PushFront("part0");
  EXPECT_EQ(path1.Str(), "part0/part1/part2");

  path2 = path1 / "part3";
  EXPECT_EQ(path2.Str(), "part0/part1/part2/part3");

  path1 /= "part3";
  EXPECT_EQ(path1.Str(), "part0/part1/part2/part3");

  EXPECT_TRUE(path1 == path2);

  path3 = path1;
  EXPECT_TRUE(path3 == path1);

  path1.Clear();
  EXPECT_TRUE(path1.Str().empty());

  URIPath path4(path3);
  EXPECT_TRUE(path4 == path3);
}

/////////////////////////////////////////////////
TEST(URITEST, URIPathString)
{
  URIPath path;
  EXPECT_FALSE(URIPath::Valid("", path));
  EXPECT_FALSE(URIPath::Valid("//", path));
  EXPECT_FALSE(URIPath::Valid(" ", path));
  EXPECT_FALSE(URIPath::Valid("?invalid", path));
  EXPECT_FALSE(URIPath::Valid("=invalid", path));
  EXPECT_FALSE(URIPath::Valid("&invalid", path));
  EXPECT_TRUE(URIPath::Valid("part1", path));
  EXPECT_TRUE(URIPath::Valid("/part1", path));
  EXPECT_TRUE(URIPath::Valid("/part1/", path));
  EXPECT_TRUE(URIPath::Valid("/part1/part2", path));
  EXPECT_TRUE(URIPath::Valid("/part1/part2/", path));

  EXPECT_FALSE(path.Load(""));
  EXPECT_FALSE(path.Load("//"));
  EXPECT_FALSE(path.Load(" "));
  EXPECT_FALSE(path.Load("?invalid"));
  EXPECT_FALSE(path.Load("=invalid"));
  EXPECT_FALSE(path.Load("&invalid"));
  EXPECT_TRUE(path.Load("part1"));
  EXPECT_TRUE(path.Load("/part1"));
  EXPECT_TRUE(path.Load("/part1/"));
  EXPECT_TRUE(path.Load("/part1/part2"));
  EXPECT_TRUE(path.Load("/part1/part2/"));

  EXPECT_THROW(URIPath(""), Exception);
  EXPECT_THROW(URIPath("//"), Exception);
  EXPECT_THROW(URIPath(" "), Exception);
  EXPECT_THROW(URIPath("?invalid"), Exception);
  EXPECT_THROW(URIPath("=invalid"), Exception);
  EXPECT_THROW(URIPath("&invalid"), Exception);
  EXPECT_NO_THROW(URIPath("part1"));
  EXPECT_NO_THROW(URIPath("/part1"));
  EXPECT_NO_THROW(URIPath("/part1/"));
  EXPECT_NO_THROW(URIPath("/part1/part2"));
  EXPECT_NO_THROW(URIPath("/part1/part2/"));
}

/////////////////////////////////////////////////
TEST(URITEST, URIQuery)
{
  URIQuery query1, query2, query3;
  EXPECT_TRUE(query1.Str().empty());

  query1.Insert("key1", "value1");
  EXPECT_EQ(query1.Str(), "?key1=value1");
  query1.Insert("key2", "value2");
  EXPECT_EQ(query1.Str(), "?key1=value1&key2=value2");

  query2 = query1;
  EXPECT_EQ(query2.Str(), query1.Str());
  EXPECT_TRUE(query2 == query1);

  query3 = query1;
  EXPECT_TRUE(query3 == query1);

  query1.Clear();
  EXPECT_TRUE(query1.Str().empty());

  URIQuery query4(query2);
  EXPECT_TRUE(query4 == query2);
}

/////////////////////////////////////////////////
TEST(URITEST, URIQueryString)
{
  URIQuery query;
  EXPECT_FALSE(URIQuery::Valid("??", query));
  EXPECT_FALSE(URIQuery::Valid("invalid?", query));
  EXPECT_FALSE(URIQuery::Valid("?invalid?", query));
  EXPECT_FALSE(URIQuery::Valid("? invalid", query));
  EXPECT_FALSE(URIQuery::Valid("?key", query));
  EXPECT_FALSE(URIQuery::Valid("?key=", query));
  EXPECT_FALSE(URIQuery::Valid("?=value", query));

  EXPECT_TRUE(URIQuery::Valid("", query));
  EXPECT_TRUE(URIQuery::Valid("?key=value", query));
  EXPECT_TRUE(URIQuery::Valid("?key=value&key2=value2", query));

  EXPECT_FALSE(query.Load("??"));
  EXPECT_FALSE(query.Load("invalid?"));
  EXPECT_FALSE(query.Load("?invalid?"));
  EXPECT_FALSE(query.Load("? invalid"));
  EXPECT_FALSE(query.Load("?key"));
  EXPECT_FALSE(query.Load("?key="));
  EXPECT_FALSE(query.Load("?=value"));

  EXPECT_TRUE(query.Load(""));
  EXPECT_TRUE(query.Load("?key=value"));
  EXPECT_TRUE(query.Load("?key=value&key2=value2"));

  EXPECT_THROW(URIQuery("??"), Exception);
  EXPECT_THROW(URIQuery("invalid?"), Exception);
  EXPECT_THROW(URIQuery("?invalid?"), Exception);
  EXPECT_THROW(URIQuery("? invalid"), Exception);
  EXPECT_THROW(URIQuery("?key"), Exception);
  EXPECT_THROW(URIQuery("?key="), Exception);
  EXPECT_THROW(URIQuery("?=value"), Exception);

  EXPECT_NO_THROW(URIQuery(""));
  EXPECT_NO_THROW(URIQuery("?key=value"));
  EXPECT_NO_THROW(URIQuery("?key=value&key2=value2"));
}

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

  uri.Query().Insert("p", "v");
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
TEST(URITEST, URIString)
{
  URI uri;
  EXPECT_FALSE(URI::Valid("", uri));
  EXPECT_FALSE(URI::Valid("scheme", uri));
  EXPECT_FALSE(URI::Valid("scheme://", uri));
  EXPECT_FALSE(URI::Valid("scheme://?key=value", uri));
  EXPECT_FALSE(URI::Valid("scheme://part1?keyvalue", uri));
  EXPECT_TRUE(URI::Valid("scheme://part1", uri));
  EXPECT_TRUE(URI::Valid("scheme://part1/part2", uri));
  EXPECT_TRUE(URI::Valid("scheme://part1?key=value", uri));
  EXPECT_TRUE(URI::Valid("scheme://part1/part2?k1=v1&k2=v2", uri));

  EXPECT_FALSE(uri.Load(""));
  EXPECT_FALSE(uri.Load("scheme"));
  EXPECT_FALSE(uri.Load("scheme://"));
  EXPECT_FALSE(uri.Load("scheme://?key=value"));
  EXPECT_FALSE(uri.Load("scheme://part1?keyvalue"));
  EXPECT_TRUE(uri.Load("scheme://part1"));
  EXPECT_TRUE(uri.Load("scheme://part1/part2"));
  EXPECT_TRUE(uri.Load("scheme://part1?key=value"));
  EXPECT_TRUE(uri.Load("scheme://part1/part2?k1=v1&k2=v2"));

  EXPECT_THROW(URI(""), Exception);
  EXPECT_THROW(URI("scheme"), Exception);
  EXPECT_THROW(URI("scheme://"), Exception);
  EXPECT_THROW(URI("scheme://?key=value"), Exception);
  EXPECT_THROW(URI("scheme://part1?keyvalue"), Exception);
  EXPECT_NO_THROW(URI("scheme://part1"));
  EXPECT_NO_THROW(URI("scheme://part1/part2"));
  EXPECT_NO_THROW(URI("scheme://part1?key=value"));
  EXPECT_NO_THROW(URI("scheme://part1/part2?k1=v1&k2=v2"));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
