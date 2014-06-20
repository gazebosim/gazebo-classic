/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/Color.hh"

using namespace gazebo;

TEST(Color, Color)
{
  common::Color clr(.1, .2, .3, 1.0);
  EXPECT_FLOAT_EQ(0.1f, clr.r);
  EXPECT_FLOAT_EQ(0.2f, clr.g);
  EXPECT_FLOAT_EQ(0.3f, clr.b);
  EXPECT_FLOAT_EQ(1.0f, clr.a);

  clr.Reset();
  EXPECT_FLOAT_EQ(0.0f, clr.r);
  EXPECT_FLOAT_EQ(0.0f, clr.g);
  EXPECT_FLOAT_EQ(0.0f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  clr.SetFromHSV(0, 0.5, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr.r);
  EXPECT_FLOAT_EQ(0.5f, clr.g);
  EXPECT_FLOAT_EQ(0.5f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  EXPECT_TRUE(clr.GetAsHSV() == math::Vector3(6, 0.5, 1));

  clr.SetFromHSV(60, 0.0, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr.r);
  EXPECT_FLOAT_EQ(1.0f, clr.g);
  EXPECT_FLOAT_EQ(1.0f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  clr.SetFromHSV(120, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.r);
  EXPECT_FLOAT_EQ(1.0f, clr.g);
  EXPECT_FLOAT_EQ(0.5f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  clr.SetFromHSV(180, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.r);
  EXPECT_FLOAT_EQ(1.0f, clr.g);
  EXPECT_FLOAT_EQ(1.0f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  clr.SetFromHSV(240, 0.5, 1.0);
  EXPECT_FLOAT_EQ(0.5f, clr.r);
  EXPECT_FLOAT_EQ(0.5f, clr.g);
  EXPECT_FLOAT_EQ(1.0f, clr.b);
  EXPECT_FLOAT_EQ(0.0f, clr.a);

  clr.SetFromHSV(300, 0.5, 1.0);
  EXPECT_FLOAT_EQ(1.0f, clr[0]);
  EXPECT_FLOAT_EQ(0.5f, clr[1]);
  EXPECT_FLOAT_EQ(1.0f, clr[2]);
  EXPECT_FLOAT_EQ(0.0f, clr[3]);
  EXPECT_FLOAT_EQ(0.0f, clr[4]);

  clr.r = 0.1;
  clr.g = 0.2;
  clr.b = 0.3;
  clr.a = 0.4;
  EXPECT_FLOAT_EQ(0.1f, clr[0]);
  EXPECT_FLOAT_EQ(0.2f, clr[1]);
  EXPECT_FLOAT_EQ(0.3f, clr[2]);
  EXPECT_FLOAT_EQ(0.4f, clr[3]);

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr = clr + 0.2;
  EXPECT_TRUE(clr == common::Color(0.3, 0.4, 0.5, 0.6));

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr += common::Color(0.2, 0.2, 0.2, 0.2);
  EXPECT_TRUE(clr == common::Color(0.3, 0.4, 0.5, 0.6));


  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr = clr - 0.1;
  EXPECT_TRUE(clr == common::Color(0.0, 0.1, 0.2, 0.3));

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr -= common::Color(0.1, 0.1, 0.1, 0.1);
  EXPECT_TRUE(clr == common::Color(0.0, 0.1, 0.2, 0.3));


  clr.Set(1, 1, 1, 1.);
  clr = clr / 1.6;
  EXPECT_TRUE(clr == common::Color(0.625, 0.625, 0.625, 0.625));

  clr.Set(1, 1, 1, 1);
  clr /= common::Color(1, 1, 1, 1);
  EXPECT_TRUE(clr == common::Color(1, 1, 1, 1));


  clr.Set(.1, .2, .3, .4);
  clr = clr * .1;
  EXPECT_TRUE(clr == common::Color(0.01, 0.02, 0.03, 0.04));

  clr.Set(.1, .2, .3, .4);
  clr *= common::Color(0.1, 0.1, 0.1, 0.1);
  EXPECT_TRUE(clr == common::Color(0.01, 0.02, 0.03, 0.04));


  clr.SetFromYUV(0.5, 0.2, 0.8);
  EXPECT_TRUE(math::equal(0.00553f, clr.r, 1e-3f));
  EXPECT_TRUE(math::equal(0.0f, clr.g));
  EXPECT_TRUE(math::equal(0.9064f, clr.b, 1e-3f));
  EXPECT_TRUE(math::equal(0.04f, clr.a));

  EXPECT_TRUE(clr.GetAsYUV() == math::Vector3(0.104985, 0.95227, 0.429305));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) + common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.00431373f, clr.r));
  EXPECT_TRUE(math::equal(0.3f, clr.g));
  EXPECT_TRUE(math::equal(0.9f, clr.b));
  EXPECT_TRUE(math::equal(2.0f, clr.a));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) - common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.9f, clr.r));
  EXPECT_TRUE(math::equal(0.0f, clr.g));
  EXPECT_TRUE(math::equal(0.1f, clr.b));
  EXPECT_TRUE(math::equal(0.0f, clr.a));

  clr = common::Color(0.5, 0.2, 0.4, 0.6) / 2.0;
  EXPECT_TRUE(math::equal(0.25f, clr.r));
  EXPECT_TRUE(math::equal(0.1f, clr.g));
  EXPECT_TRUE(math::equal(0.2f, clr.b));
  EXPECT_TRUE(math::equal(0.3f, clr.a));
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
