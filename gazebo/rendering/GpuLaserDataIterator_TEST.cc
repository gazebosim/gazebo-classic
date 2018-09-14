/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <type_traits>
#include <gtest/gtest.h>

#include "GpuLaserDataIterator.hh"

using gazebo::rendering::GpuLaserDataIterator;

// Test hook to access protected parts of iterator
class GpuLaserDataIteratorTest : public ::testing::Test
{
};

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanConstruct)
{
  const unsigned int index = 5;
  const float readings[] = {0, 1, 2};
  const unsigned int skip = 6;
  const unsigned int rangeOffset = 7;
  const unsigned int intenOffset = 8;
  const unsigned int horizontalResolution = 9;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      index, readings, skip, rangeOffset, intenOffset, horizontalResolution);

  EXPECT_EQ(index, it.index);
  EXPECT_EQ(readings, it.data);
  EXPECT_EQ(skip, it.skip);
  EXPECT_EQ(rangeOffset, it.rangeOffset);
  EXPECT_EQ(intenOffset, it.intensityOffset);
  EXPECT_EQ(horizontalResolution, it.horizontalResolution);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanGetFirst)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      0, readings, skip, rangeOffset, intenOffset, hRes);

  EXPECT_DOUBLE_EQ(readings[0], it->range);
  EXPECT_DOUBLE_EQ(readings[1], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanGetMiddle)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      1, readings, skip, rangeOffset, intenOffset, hRes);

  EXPECT_DOUBLE_EQ(readings[3], it->range);
  EXPECT_DOUBLE_EQ(readings[4], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanGetLast)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      2, readings, skip, rangeOffset, intenOffset, hRes);

  EXPECT_DOUBLE_EQ(readings[6], it->range);
  EXPECT_DOUBLE_EQ(readings[7], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, OneBeamSensor)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  for (unsigned int i = 0; i < hRes; ++i)
  {
    GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
        i, readings, skip, rangeOffset, intenOffset, hRes);
    EXPECT_EQ(i, it->reading);
  }
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, threeBeamSensor)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 1;
  const unsigned int vRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  for (unsigned int i = 0; i < vRes; ++i)
  {
    GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
        i, readings, skip, rangeOffset, intenOffset, hRes);
    EXPECT_EQ(i, it->beam);
  }
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, threeByThreeSensor)
{
  const float readings[] = { 0,  1,  3,  4,  6,  7,
                             9, 10, 12, 13, 15, 16,
                            18, 19, 21, 22, 24, 25};
  const unsigned int hRes = 3;
  const unsigned int skip = 2;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;
  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      0, readings, skip, rangeOffset, intenOffset, hRes);

  // 0
  EXPECT_DOUBLE_EQ(readings[0], it->range);
  EXPECT_DOUBLE_EQ(readings[1], it->intensity);
  EXPECT_EQ(0u, it->beam);
  EXPECT_EQ(0u, it->reading);
  ++it;
  // 1
  EXPECT_DOUBLE_EQ(readings[2], it->range);
  EXPECT_DOUBLE_EQ(readings[3], it->intensity);
  EXPECT_EQ(0u, it->beam);
  EXPECT_EQ(1u, it->reading);
  ++it;
  // 2
  EXPECT_DOUBLE_EQ(readings[4], it->range);
  EXPECT_DOUBLE_EQ(readings[5], it->intensity);
  EXPECT_EQ(0u, it->beam);
  EXPECT_EQ(2u, it->reading);
  ++it;
  // 3
  EXPECT_DOUBLE_EQ(readings[6], it->range);
  EXPECT_DOUBLE_EQ(readings[7], it->intensity);
  EXPECT_EQ(1u, it->beam);
  EXPECT_EQ(0u, it->reading);
  ++it;
  // 4
  EXPECT_DOUBLE_EQ(readings[8], it->range);
  EXPECT_DOUBLE_EQ(readings[9], it->intensity);
  EXPECT_EQ(1u, it->beam);
  EXPECT_EQ(1u, it->reading);
  ++it;
  // 5
  EXPECT_DOUBLE_EQ(readings[10], it->range);
  EXPECT_DOUBLE_EQ(readings[11], it->intensity);
  EXPECT_EQ(1u, it->beam);
  EXPECT_EQ(2u, it->reading);
  ++it;
  // 6
  EXPECT_DOUBLE_EQ(readings[12], it->range);
  EXPECT_DOUBLE_EQ(readings[13], it->intensity);
  EXPECT_EQ(2u, it->beam);
  EXPECT_EQ(0u, it->reading);
  ++it;
  // 7
  EXPECT_DOUBLE_EQ(readings[14], it->range);
  EXPECT_DOUBLE_EQ(readings[15], it->intensity);
  EXPECT_EQ(2u, it->beam);
  EXPECT_EQ(1u, it->reading);
  ++it;
  // 8
  EXPECT_DOUBLE_EQ(readings[16], it->range);
  EXPECT_DOUBLE_EQ(readings[17], it->intensity);
  EXPECT_EQ(2u, it->beam);
  EXPECT_EQ(2u, it->reading);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanPrefixIncrement)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      0, readings, skip, rangeOffset, intenOffset, hRes);

  auto it_reference = ++it;

  EXPECT_EQ(it_reference, it);
  EXPECT_DOUBLE_EQ(readings[3], it->range);
  EXPECT_DOUBLE_EQ(readings[4], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanPostfixIncrement)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      0, readings, skip, rangeOffset, intenOffset, hRes);

  auto it_copy = it++;

  EXPECT_NE(it_copy, it);
  EXPECT_DOUBLE_EQ(readings[3], it->range);
  EXPECT_DOUBLE_EQ(readings[4], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanPrefixDecrement)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      1, readings, skip, rangeOffset, intenOffset, hRes);

  auto it_reference = --it;

  EXPECT_EQ(it_reference, it);
  EXPECT_DOUBLE_EQ(readings[0], it->range);
  EXPECT_DOUBLE_EQ(readings[1], it->intensity);
}

/////////////////////////////////////////////////
TEST(GpuLaserDataIteratorTest, CanPostfixDecrement)
{
  const float readings[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  const unsigned int hRes = 3;
  const unsigned int skip = 3;
  const unsigned int rangeOffset = 0;
  const unsigned int intenOffset = 1;

  GpuLaserDataIterator<std::remove_reference<decltype(*this)>::type> it(
      1, readings, skip, rangeOffset, intenOffset, hRes);

  auto it_copy = it--;

  EXPECT_NE(it_copy, it);
  EXPECT_DOUBLE_EQ(readings[0], it->range);
  EXPECT_DOUBLE_EQ(readings[1], it->intensity);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
