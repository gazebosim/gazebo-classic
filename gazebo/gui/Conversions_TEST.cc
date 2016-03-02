/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Color.hh"
#include "gazebo/gui/Conversions.hh"
#include "test/util.hh"

using namespace gazebo;

class GuiConversionsTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(GuiConversionsTest, Color)
{
  // Gazebo to Qt to Gazebo
  {
    double red = 0.1;
    double green = 0.3;
    double blue = 0.5;
    double alpha = 0.7;

    double tol = 0.01;

    gazebo::common::Color color(red, green, blue, alpha);
    auto newColor = gazebo::gui::Conversions::Convert(
                    gazebo::gui::Conversions::Convert(color));

    EXPECT_NEAR(newColor.r, newColor.r, tol);
    EXPECT_NEAR(newColor.g, newColor.g, tol);
    EXPECT_NEAR(newColor.b, newColor.b, tol);
    EXPECT_NEAR(newColor.a, newColor.a, tol);
  }

  // Qt to Gazebo to Qt
  {
    int red = 100;
    int green = 150;
    int blue = 200;
    int alpha = 186;

    QColor color(red, green, blue, alpha);
    EXPECT_EQ(gazebo::gui::Conversions::Convert(
              gazebo::gui::Conversions::Convert(color)), color);
  }
}

/////////////////////////////////////////////////
TEST_F(GuiConversionsTest, Point2d)
{
  double x = -0.5;
  double y = 123;

  // Ignition to Qt to Ignition
  {
    ignition::math::Vector2d point(x, y);
    EXPECT_EQ(gazebo::gui::Conversions::Convert(
              gazebo::gui::Conversions::Convert(point)), point);
  }

  // Qt to Ignition to Qt
  {
    QPointF point(x, y);
    EXPECT_EQ(gazebo::gui::Conversions::Convert(
              gazebo::gui::Conversions::Convert(point)), point);
  }
}

/////////////////////////////////////////////////
TEST_F(GuiConversionsTest, Vector3d)
{
  double x = -0.1;
  double y = 0;
  double z = 1234;

  // Ignition to Qt to Ignition
  {
    ignition::math::Vector3d vec(x, y, z);
    EXPECT_EQ(gazebo::gui::Conversions::Convert(
              gazebo::gui::Conversions::Convert(vec)), vec);
  }

  // Qt to Ignition to Qt
  {
    QVector3D vec(x, y, z);
    EXPECT_EQ(gazebo::gui::Conversions::Convert(
              gazebo::gui::Conversions::Convert(vec)), vec);
  }
}

