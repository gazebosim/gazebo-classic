/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include "gazebo/common/MovingWindowFilter.hh"
#include "gazebo/math/Vector3.hh"

using namespace gazebo;

/////////////////////////////////////////////////
TEST(MovingWindowFilterTest, SetWindowSize)
{
  common::MovingWindowFilter<int> filterInt;

  EXPECT_EQ(filterInt.GetWindowSize(), 4u);
  EXPECT_FALSE(filterInt.GetWindowFilled());

  filterInt.SetWindowSize(10);
  EXPECT_EQ(filterInt.GetWindowSize(), 10u);
  EXPECT_FALSE(filterInt.GetWindowFilled());
}

/////////////////////////////////////////////////
TEST(MovingWindowFilterTest, FilterSomething)
{
  common::MovingWindowFilter<double> doubleMWF;
  common::MovingWindowFilter<double> doubleMWF2;
  common::MovingWindowFilter<math::Vector3> vectorMWF;

  doubleMWF.SetWindowSize(10);
  doubleMWF2.SetWindowSize(2);
  vectorMWF.SetWindowSize(40);

  for (unsigned int i = 0; i < 20; ++i)
  {
    doubleMWF.Update(static_cast<double>(i));
    doubleMWF2.Update(static_cast<double>(i));
    math::Vector3 v(1.0*static_cast<double>(i),
                    2.0*static_cast<double>(i),
                    3.0*static_cast<double>(i));
    vectorMWF.Update(v);
  }

  double sum = 0;
  for (unsigned int i = 20-10; i < 20; ++i)
    sum += static_cast<double>(i);
  EXPECT_DOUBLE_EQ(doubleMWF.Get(), sum/10.0);
  EXPECT_DOUBLE_EQ(doubleMWF2.Get(), (18.0+19.0)/2.0);

  math::Vector3 vsum;
  for (unsigned int i = 0; i < 20; ++i)
    vsum += math::Vector3(1.0*static_cast<double>(i),
                          2.0*static_cast<double>(i),
                          3.0*static_cast<double>(i));
  EXPECT_EQ(vectorMWF.Get(), vsum / 20.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
