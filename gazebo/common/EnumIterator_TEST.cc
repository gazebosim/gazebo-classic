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
#include "gazebo/common/EnumIterator.hh"
#include "test/util.hh"

using namespace gazebo;

class EnumIteratorTest : public gazebo::testing::AutoLogFixture { };

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
  "MY_TYPE_END")

/////////////////////////////////////////////////
TEST_F(EnumIteratorTest, Iterator)
{
  int i = 0;
  for (common::EnumIterator<MyType> typeIter = MY_TYPE_BEGIN, e;
       typeIter != e; ++typeIter, ++i)
  {
    EXPECT_EQ(*typeIter, i);
    if (i == 0)
      EXPECT_EQ(common::EnumId<MyType>::Str(*typeIter), "TYPE1");
    else
      EXPECT_EQ(common::EnumId<MyType>::Str(*typeIter), "TYPE2");
  }
}
