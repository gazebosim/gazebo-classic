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
#include "gazebo/common/Tokenizer.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
TEST(URITEST, TokenizerTest)
{
  Tokenizer tk1("abc/def");
  auto tokens = tk1.Split("/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  Tokenizer tk2("abc/def/");
  tokens = tk2.Split("/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  Tokenizer tk3("//abc/def///");
  tokens = tk3.Split("/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  Tokenizer tk4("abc");
  tokens = tk4.Split("/");
  ASSERT_EQ(tokens.size(), 1u);
  EXPECT_EQ(tokens.at(0), "abc");

  Tokenizer tk5("//abc/def::123::567///");
  tokens = tk5.Split("/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def::123::567");
  tokens = tk5.Split("::");
  ASSERT_EQ(tokens.size(), 3u);
  EXPECT_EQ(tokens.at(0), "//abc/def");
  EXPECT_EQ(tokens.at(1), "123");
  EXPECT_EQ(tokens.at(2), "567///");
}
