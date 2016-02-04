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

#include "gazebo/math/Matrix3.hh"

using namespace gazebo;

class Matrix3Test : public ::testing::Test { };

TEST_F(Matrix3Test, Matrix3)
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

  EXPECT_THROW(matrix.SetCol(3, math::Vector3(1, 1, 1)), std::string);
}

TEST_F(Matrix3Test, Multiplication)
{
  {
    // Multiply arbitrary matrix by zeros of different sizes
    math::Matrix3 matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);

    // Scalar 0
    EXPECT_EQ(math::Matrix3::ZERO, matrix * 0);
    EXPECT_EQ(math::Matrix3::ZERO, 0 * matrix);

    // Vector3::Zero
    EXPECT_EQ(math::Vector3::Zero, matrix * math::Vector3::Zero);
    // left multiply with Vector3 not implemented

    // Matrix3::ZERO
    EXPECT_EQ(math::Matrix3::ZERO, matrix * math::Matrix3::ZERO);
    EXPECT_EQ(math::Matrix3::ZERO, math::Matrix3::ZERO * matrix);
  }

  {
    // Multiply arbitrary matrix by identity values
    math::Matrix3 matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);

    // scalar 1.0
    EXPECT_EQ(matrix, matrix * 1.0);
    EXPECT_EQ(matrix, 1.0 * matrix);

    // Vector3::Unit[X|Y|Z]
    EXPECT_EQ(math::Vector3(matrix[0][0], matrix[1][0], matrix[2][0]),
              matrix * math::Vector3::UnitX);
    EXPECT_EQ(math::Vector3(matrix[0][1], matrix[1][1], matrix[2][1]),
              matrix * math::Vector3::UnitY);
    EXPECT_EQ(math::Vector3(matrix[0][2], matrix[1][2], matrix[2][2]),
              matrix * math::Vector3::UnitZ);

    // Matrix3::IDENTITY
    EXPECT_EQ(matrix, matrix * math::Matrix3::IDENTITY);
    EXPECT_EQ(matrix, math::Matrix3::IDENTITY * matrix);
  }

  {
    // Multiply arbitrary matrix by itself
    math::Matrix3 matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    math::Matrix3 matrix2(30,  36,  42,
                          66,  81,  96,
                         102, 126, 150);

    EXPECT_EQ(matrix * matrix, matrix2);
  }
}

TEST_F(Matrix3Test, Inverse)
{
  // Inverse of identity matrix is itself
  EXPECT_EQ(math::Matrix3::IDENTITY, math::Matrix3::IDENTITY.Inverse());

  // Matrix multiplied by its inverse results in the identity matrix
  math::Matrix3 matrix1(-2, 4, 0, 0.1, 9, 55, -7, 1, 26);
  math::Matrix3 matrix2 = matrix1.Inverse();
  EXPECT_EQ(matrix1 * matrix2, math::Matrix3::IDENTITY);
  EXPECT_EQ(matrix2 * matrix1, math::Matrix3::IDENTITY);

  // Inverse of inverse results in the same matrix
  EXPECT_EQ((matrix1.Inverse()).Inverse(), matrix1);

  // Invert multiplication by scalar
  double scalar = 2.5;
  EXPECT_EQ((matrix1 * scalar).Inverse(), matrix1.Inverse() * (1.0/scalar));
}
