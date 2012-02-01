/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <string.h>
#include "msgs/msgs.h"
#include "ServerFixture.hh"

using namespace gazebo;
class MathTest : public ServerFixture
{
};

TEST_F(MathTest, Box)
{
  {
    math::Box box;
    EXPECT_TRUE(box.min == math::Vector3(0, 0, 0));
    EXPECT_TRUE(box.max == math::Vector3(0, 0, 0));
  }

  {
    math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
    EXPECT_TRUE(box.min == math::Vector3(0, 1, 2));
    EXPECT_TRUE(box.max == math::Vector3(1, 2, 3));
  }

  {
    math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
    math::Box box1(box);
    EXPECT_TRUE(box1.min == box.min);
    EXPECT_TRUE(box1.max == box.max);
  }

  math::Box box(math::Vector3(0, 1, 2), math::Vector3(1, 2, 3));
  EXPECT_TRUE(box.GetXLength() == 1);
  EXPECT_TRUE(box.GetYLength() == 1);
  EXPECT_TRUE(box.GetZLength() == 1);
  EXPECT_TRUE(box.GetSize() == math::Vector3(1, 1, 1));
  EXPECT_TRUE(box.GetCenter() == math::Vector3(0.5, 1.5, 2.5));

  box.Merge(math::Box(math::Vector3(-1, -1, -1), math::Vector3(2, 2, 2)));
  EXPECT_TRUE(box == math::Box(math::Vector3(-1, -1, -1),
                               math::Vector3(2, 2, 3)));

  box = math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(3, 3, 3)));

  box += math::Box(math::Vector3(2, 2, 2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(1, 1, 1), math::Vector3(4, 4, 4)));

  box = box + math::Box(math::Vector3(-2, -2, -2), math::Vector3(4, 4, 4));
  EXPECT_TRUE(box == math::Box(math::Vector3(-2, -2, -2),
                               math::Vector3(4, 4, 4)));
}

TEST_F(MathTest, Plane)
{
  {
    math::Plane plane;
    EXPECT_TRUE(math::equal(plane.d, 0.0));
    EXPECT_TRUE(plane.normal == math::Vector3());
    EXPECT_TRUE(plane.size == math::Vector2d(0, 0));
  }

  {
    math::Plane plane(math::Vector3(0, 0, 1), math::Vector2d(2, 3), 2.0);
    EXPECT_TRUE(math::equal(plane.d, 2.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 0, 1));
    EXPECT_TRUE(plane.size == math::Vector2d(2, 3));

    plane.Set(math::Vector3(1, 0, 0), math::Vector2d(1, 1), 1.0);
    EXPECT_TRUE(math::equal(plane.d, 1.0));
    EXPECT_TRUE(plane.normal == math::Vector3(1, 0, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(1, 1));

    plane = math::Plane(math::Vector3(0, 1, 0), math::Vector2d(4, 4), 5.0);
    EXPECT_TRUE(math::equal(plane.d, 5.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 1, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(4, 4));
  }
}

TEST_F(MathTest, Matrix3)
{
  {
    math::Matrix3 matrix;
    EXPECT_TRUE(matrix == math::Matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0));
  }

  {
    math::Matrix3 matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_TRUE(matrix == math::Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9));

    math::Matrix3 matrix1(matrix);
    EXPECT_TRUE(matrix1 == math::Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9));
  }

  math::Matrix3 matrix;
  matrix.SetFromAxes(math::Vector3(1, 1, 1), math::Vector3(2, 2, 2),
                     math::Vector3(3, 3, 3));
  EXPECT_TRUE(matrix == math::Matrix3(1, 2, 3, 1, 2, 3, 1, 2, 3));

  matrix.SetFromAxis(math::Vector3(1, 1, 1), M_PI);
  EXPECT_TRUE(matrix == math::Matrix3(1, 2, 2, 2, 1, 2, 2, 2, 1));

  matrix.SetCol(0, math::Vector3(3, 4, 5));
  EXPECT_TRUE(matrix == math::Matrix3(3, 2, 2, 4, 1, 2, 5, 2, 1));

  EXPECT_THROW(matrix.SetCol(3, math::Vector3(1, 1, 1)), common::Exception);
}

TEST_F(MathTest, Angle)
{
  math::Angle angle1;
  EXPECT_TRUE(math::equal(0.0, angle1.GetAsRadian()));

  angle1.SetFromDegree(180.0);
  EXPECT_TRUE(angle1 == M_PI);

  angle1 = math::Angle(0.1) - math::Angle(0.3);
  EXPECT_TRUE(angle1 == -0.2);

  math::Angle angle(0.5);
  EXPECT_TRUE(math::equal(0.5, angle.GetAsRadian()));

  angle.SetFromRadian(M_PI);
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.GetAsDegree()));

  angle.Normalize();
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.GetAsDegree()));

  angle = math::Angle(0.1) + math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.3, angle.GetAsRadian()));

  angle = math::Angle(0.1) * math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.02, angle.GetAsRadian()));

  angle = math::Angle(0.1) / math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.5, angle.GetAsRadian()));

  angle -= math::Angle(0.1);
  EXPECT_TRUE(math::equal(0.4, angle.GetAsRadian()));

  angle += math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.6, angle.GetAsRadian()));

  angle *= math::Angle(0.5);
  EXPECT_TRUE(math::equal(0.3, angle.GetAsRadian()));

  angle /= math::Angle(0.1);
  EXPECT_TRUE(math::equal(3, angle.GetAsRadian()));
  EXPECT_TRUE(angle == math::Angle(3));
  EXPECT_TRUE(angle != math::Angle(2));
  EXPECT_TRUE(angle < math::Angle(4));
  EXPECT_TRUE(angle <= math::Angle(3));
  EXPECT_TRUE(angle > math::Angle(2));
  EXPECT_TRUE(angle >= math::Angle(3));
}
  
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
