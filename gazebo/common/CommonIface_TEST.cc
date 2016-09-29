/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <fstream>
#include <sstream>

#include "gazebo/common/CommonIface.hh"
#include "test/util.hh"

using namespace gazebo;

class CommonIface_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test CommonIface::GetSHA1
TEST_F(CommonIface_TEST, GetSHA1)
{
  // Do not forget to update 'precomputedSHA1' if you modify the SHA1 input.
  std::string precomputedSHA1;
  std::string computedSHA1;
  std::string s;

  // Compute the SHA1 of the vector
  std::vector<float> v;
  for (int i = 0; i < 100; ++i)
    v.push_back(i);

  computedSHA1 = common::get_sha1<std::vector<float> >(v);
  precomputedSHA1 = "913283ec8502ba1423d38a7ea62cb8e492e87b23";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of a string
  s = "Marty McFly: Wait a minute, Doc. Ah... Are you telling me that you"
      " built a time machine... out of a DeLorean?\n"
      "Dr. Emmett Brown: The way I see it, if you're gonna build a time"
      " machine into a car, why not do it with some style?";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "a370ddc4d61d936b2bb40f98bae061dc15fd8923";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of an empty string
  s = "";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "da39a3ee5e6b4b0d3255bfef95601890afd80709";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute a bunch of SHA1's to verify consistent length
  for (unsigned i = 0; i < 100; ++i)
  {
    std::stringstream stream;
    stream << i << '\n';
    std::string sha = common::get_sha1<std::string>(stream.str());
    EXPECT_EQ(sha.length(), 40u);
  }
}

/////////////////////////////////////////////////
/// \brief Test the string tokenizer split() function.
TEST_F(CommonIface_TEST, split)
{
  auto tokens = common::split("abc/def", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("abc/def/", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("//abc/def///", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("abc", "/");
  ASSERT_EQ(tokens.size(), 1u);
  EXPECT_EQ(tokens.at(0), "abc");

  tokens = common::split("//abc/def::123::567///", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def::123::567");
  tokens = common::split("//abc/def::123::567///", "::");
  ASSERT_EQ(tokens.size(), 3u);
  EXPECT_EQ(tokens.at(0), "//abc/def");
  EXPECT_EQ(tokens.at(1), "123");
  EXPECT_EQ(tokens.at(2), "567///");
}

/////////////////////////////////////////////////
/// \brief Test file operations
TEST_F(CommonIface_TEST, fileOps)
{
  EXPECT_FALSE(common::cwd().empty());
  EXPECT_TRUE(common::exists(common::cwd()));
  EXPECT_TRUE(common::isDirectory(common::cwd()));

  EXPECT_TRUE(common::isFile(__FILE__));
  EXPECT_FALSE(common::isDirectory(__FILE__));

  std::ofstream tmpOut("test.tmp");
  tmpOut << "Output" << std::endl;
  tmpOut.close();

  EXPECT_TRUE(common::copyFile("test.tmp", "test2.tmp"));
  EXPECT_TRUE(common::exists("test.tmp"));
  EXPECT_TRUE(common::exists("test2.tmp"));

  std::ifstream testIn("test.tmp");
  std::string testInContent((std::istreambuf_iterator<char>(testIn)),
                            (std::istreambuf_iterator<char>()));

  std::ifstream test2In("test2.tmp");
  std::string test2InContent((std::istreambuf_iterator<char>(test2In)),
                             (std::istreambuf_iterator<char>()));


  EXPECT_EQ(testInContent, test2InContent);

  EXPECT_TRUE(common::moveFile("test2.tmp", "test3.tmp"));
  EXPECT_FALSE(common::exists("test2.tmp"));
  EXPECT_TRUE(common::exists("test3.tmp"));

  std::ifstream test3In("test3.tmp");
  std::string test3InContent((std::istreambuf_iterator<char>(test3In)),
                             (std::istreambuf_iterator<char>()));

  EXPECT_EQ(testInContent, test3InContent);

  EXPECT_FALSE(common::copyFile("test3.tmp", "test3.tmp"));
  EXPECT_FALSE(common::copyFile("test3.tmp", "./test3.tmp"));

  std::remove("test.tmp");

  // This file shouldn't exist, but we'll try to remove just in case the
  // test failed.
  std::remove("test2.tmp");

  std::remove("test3.tmp");
}

/////////////////////////////////////////////////
/// \brief Test file operations
TEST_F(CommonIface_TEST, moreFileOps)
{
  EXPECT_FALSE(common::copyFile("__wrong__.tmp", "test2.tmp"));
  EXPECT_TRUE(!common::exists("test2.tmp"));
  EXPECT_FALSE(common::copyFile("test.tmp", "__wrong_dir__/__wrong__.tmp"));
  EXPECT_TRUE(!common::exists("__wrong_dir__"));

  EXPECT_FALSE(common::moveFile("__wrong__.tmp", "test3.tmp"));
  EXPECT_TRUE(!common::exists("test3.tmp"));
  EXPECT_FALSE(common::moveFile("test2.tmp", "__wrong_dir__/__wrong__.tmp"));
  EXPECT_TRUE(!common::exists("__wrong_dir__"));
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, replaceAll)
{
  std::string orig = "//abcd/efg///ijk////lm/////////////nop//";

  // No change should occur
  std::string result = common::replaceAll(orig, "//", "//");
  EXPECT_EQ(result, orig);
  result = common::replaceAll(orig, "/", "/");
  EXPECT_EQ(result, orig);

  result = common::replaceAll(orig, "//", "::");
  EXPECT_EQ(result, "::abcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, "a", "aaaa");
  EXPECT_EQ(result, "::aaaabcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, "::aaaa", " ");
  EXPECT_EQ(result, " bcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, " ", "_");
  EXPECT_EQ(result, "_bcd/efg::/ijk::::lm::::::::::::/nop::");

  std::string spaces = " 1  2   3    4     5      6       7 ";
  result = common::replaceAll(spaces, " ", "_");
  EXPECT_EQ(result, "_1__2___3____4_____5______6_______7_");

  result = common::replaceAll(spaces, "  ", "_");
  EXPECT_EQ(result, " 1_2_ 3__4__ 5___6___ 7 ");

  std::string test = "12345555675";
  common::replaceAll(test, test, "5", "*");
  EXPECT_EQ(test, "1234****67*");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
