/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "test_config.h"
#include "gazebo/common/EnumIface.hh"
#include "test/util.hh"

using namespace gazebo;

class EnumIfaceTest : public gazebo::testing::AutoLogFixture { };

enum MyType
{
  MY_TYPE_BEGIN = 0,
  TYPE1 = MY_TYPE_BEGIN,
  TYPE2 = 1,
  MY_TYPE_END
};

GZ_ENUM(MyType, MY_TYPE_BEGIN, MY_TYPE_END,
  "TYPE1",
  "TYPE2",
  "MY_TYPE_END"
)

/////////////////////////////////////////////////
TEST_F(EnumIfaceTest, StringCoversion)
{
  MyType type;

  // Set value from string
  common::EnumIface<MyType>::Set("TYPE1", type);
  EXPECT_EQ(type, TYPE1);

  // Convert value to string
  std::string typeStr = common::EnumIface<MyType>::Str(type);
  EXPECT_EQ(typeStr, "TYPE1");
}

/////////////////////////////////////////////////
TEST_F(EnumIfaceTest, Iterator)
{
  common::EnumIterator<MyType> end;
  EXPECT_EQ(end.Value(), MY_TYPE_END);
  EXPECT_EQ(common::EnumIterator<MyType>::End(), MY_TYPE_END);

  common::EnumIterator<MyType> begin(MY_TYPE_BEGIN);
  EXPECT_EQ(*begin, MY_TYPE_BEGIN);
  EXPECT_EQ(common::EnumIterator<MyType>::Begin(), MY_TYPE_BEGIN);

  int i = 0;

  // Prefix ++ operator
  for (common::EnumIterator<MyType> typeIter = MY_TYPE_BEGIN;
       typeIter != end; ++typeIter, ++i)
  {
    EXPECT_EQ(*typeIter, i);
    if (i == 0)
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE1");
    else
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE2");
  }

  // Postfix ++ operator
  i = 0;
  for (common::EnumIterator<MyType> typeIter = MY_TYPE_BEGIN;
       typeIter != end; typeIter++, ++i)
  {
    EXPECT_EQ(typeIter.Value(), i);
    if (i == 0)
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE1");
    else
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE2");
  }

  // Prefix -- operator
  i = MY_TYPE_END - 1;
  for (common::EnumIterator<MyType> typeIter = --end;
       typeIter != begin; --typeIter, --i)
  {
    EXPECT_EQ(*typeIter, i);
    if (i == 0)
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE1");
    else
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE2");
  }

  // Postfix -- operator
  i = MY_TYPE_END - 1;
  for (common::EnumIterator<MyType> typeIter = --end;
       typeIter != begin; typeIter--, --i)
  {
    EXPECT_EQ(*typeIter, i);
    if (i == 0)
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE1");
    else
      EXPECT_EQ(common::EnumIface<MyType>::Str(*typeIter), "TYPE2");
  }
}
