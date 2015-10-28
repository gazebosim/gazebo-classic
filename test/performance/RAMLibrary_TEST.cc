/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "RAMLibrary.hh"

using namespace gazebo::test::memory;

TEST(RAMLibrary, GetMemoryAvailable_NoZero)
{
  ASSERT_GT(GetMemoryAvailable(), 0.0);
}

TEST(RAMLibrary, GetTotalMemory_NoZero)
{
  ASSERT_GT(GetTotalMemory(), 0.0);
}

TEST(RAMLibrary, GetTotalMemory_GreaterThanAvailable)
{
  ASSERT_GT(GetTotalMemory(), GetMemoryAvailable());
}

TEST(RAMLibrary, IsMemoryAvailable_OneMegabyte_True)
{
  ASSERT_TRUE(IsMemoryAvailable(1));
}

TEST(RAMLibrary, IsMemoryAvailable_TotalMem_False)
{
  ASSERT_FALSE(IsMemoryAvailable(GetTotalMemory()));
}

TEST(RAMLibrary, IsMemoryAvailable_CrazyHighNumber_False)
{
  ASSERT_FALSE(IsMemoryAvailable(9999999999));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
