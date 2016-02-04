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

#include <gtest/gtest.h>

#include "gazebo/math/Box.hh"

using namespace gazebo;

class BoxTest : public ::testing::Test { };

class ExampleBox : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      box = math::Box(math::Vector3(0, -1, 2), math::Vector3(1, -2, 3));
    }

    math::Box box;
};

TEST_F(BoxTest, EmptyConstructor)
{
  math::Box box;
  EXPECT_TRUE(box.min == math::Vector3(0, 0, 0));
  EXPECT_TRUE(box.max == math::Vector3(0, 0, 0));
}

TEST_F(ExampleBox, Constructor)
{
  EXPECT_TRUE(box.min == math::Vector3(0, -2, 2));
  EXPECT_TRUE(box.max == math::Vector3(1, -1, 3));
}

TEST_F(ExampleBox, CopyConstructor)
{
  math::Box box1(box);
  EXPECT_TRUE(box1.min == box.min);
  EXPECT_TRUE(box1.max == box.max);
}

TEST_F(ExampleBox, GetLength)
{
  EXPECT_DOUBLE_EQ(box.GetXLength(), 1);
  EXPECT_DOUBLE_EQ(box.GetYLength(), 1);
  EXPECT_DOUBLE_EQ(box.GetZLength(), 1);
}

TEST_F(ExampleBox, GetSize)
{
  EXPECT_TRUE(box.GetSize() == math::Vector3(1, 1, 1));
}

TEST_F(ExampleBox, GetCenter)
{
  EXPECT_TRUE(box.GetCenter() == math::Vector3(0.5, -1.5, 2.5));
}

TEST_F(ExampleBox, Merge)
{
  box.Merge(math::Box(math::Vector3(-1, -1, -1), math::Vector3(2, 2, 2)));
  EXPECT_TRUE(box == math::Box(math::Vector3(-1, -2, -1),
                               math::Vector3(2, 2, 3)));
}

TEST_F(BoxTest, OperatorEqual)
{
  math::Box box = math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3)));
}

TEST_F(BoxTest, OperatorPlusEqual)
{
  math::Box box = math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3));
  box += math::Box(math::Vector3(2, 2, 2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(4, 4, 4)));
}

TEST_F(BoxTest, OperatorPlus)
{
  math::Box box = math::Box(math::Vector3(1, 1, 1), math::Vector3(4, 4, 4));
  box = box + math::Box(math::Vector3(-2, -2, -2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(-2, -2, -2),
                               math::Vector3(4, 4, 4)));
}

/////////////////////////////////////////////////
TEST_F(BoxTest, Contains)
{
  math::Box box = math::Box(math::Vector3(0, 0, 0), math::Vector3(1, 1, 1));

  EXPECT_TRUE(box.Contains(math::Vector3(0, 0, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3(0, 0, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3(0, 1, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3(1, 1, 1)));
  EXPECT_TRUE(box.Contains(math::Vector3(1, 1, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3(1, 0, 0)));
  EXPECT_TRUE(box.Contains(math::Vector3(0.5, 0.5, 0.5)));

  EXPECT_FALSE(box.Contains(math::Vector3(0, 0, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3(0, -1, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3(-1, -1, -1)));
  EXPECT_FALSE(box.Contains(math::Vector3(-1, -1, 0)));
  EXPECT_FALSE(box.Contains(math::Vector3(-1, 0, 0)));

  EXPECT_FALSE(box.Contains(math::Vector3(0.5, 0.5, -0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3(0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3(-0.5, 0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3(-0.5, -0.5, 0.5)));
  EXPECT_FALSE(box.Contains(math::Vector3(-0.5, -0.5, -0.5)));

  EXPECT_FALSE(box.Contains(math::Vector3(0, 0, -0.01)));
  EXPECT_FALSE(box.Contains(math::Vector3(0, -0.01, 0)));
  EXPECT_FALSE(box.Contains(math::Vector3(-0.01, 0, 0)));
}
