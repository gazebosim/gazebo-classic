/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "test/util.hh"

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class Color : public gazebo::testing::AutoLogFixture { };

TEST_F(Color, Color)
{
  common::Color clr(.1f, .2f, .3f, 1.0f);
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

  EXPECT_TRUE(clr.HSV() == ignition::math::Vector3d(6, 0.5, 1));

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

  clr.r = 0.1f;
  clr.g = 0.2f;
  clr.b = 0.3f;
  clr.a = 0.4f;
  EXPECT_FLOAT_EQ(0.1f, clr[0]);
  EXPECT_FLOAT_EQ(0.2f, clr[1]);
  EXPECT_FLOAT_EQ(0.3f, clr[2]);
  EXPECT_FLOAT_EQ(0.4f, clr[3]);

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr = clr + 0.2f;
  EXPECT_TRUE(clr == common::Color(0.3f, 0.4f, 0.5f, 0.6f));

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr += common::Color(0.2f, 0.2f, 0.2f, 0.2f);
  EXPECT_TRUE(clr == common::Color(0.3f, 0.4f, 0.5f, 0.6f));


  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr = clr - 0.1f;
  EXPECT_TRUE(clr == common::Color(0.0f, 0.1f, 0.2f, 0.3f));

  clr.Set(0.1f, 0.2f, 0.3f, 0.4f);
  clr -= common::Color(0.1f, 0.1f, 0.1f, 0.1f);
  EXPECT_TRUE(clr == common::Color(0.0f, 0.1f, 0.2f, 0.3f));


  clr.Set(1, 1, 1, 1.);
  clr = clr / 1.6f;
  EXPECT_TRUE(clr == common::Color(0.625, 0.625, 0.625, 0.625));

  clr.Set(1, 1, 1, 1);
  clr /= common::Color(1, 1, 1, 1);
  EXPECT_TRUE(clr == common::Color(1, 1, 1, 1));


  clr.Set(.1f, .2f, .3f, .4f);
  clr = clr * .1f;
  EXPECT_TRUE(clr == common::Color(0.01f, 0.02f, 0.03f, 0.04f));

  clr.Set(.1f, .2f, .3f, .4f);
  clr *= common::Color(0.1f, 0.1f, 0.1f, 0.1f);
  EXPECT_TRUE(clr == common::Color(0.01f, 0.02f, 0.03f, 0.04f));


  clr.SetFromYUV(0.5f, 0.2f, 0.8f);
  EXPECT_TRUE(ignition::math::equal(0.00553f, clr.r, 1e-3f));
  EXPECT_TRUE(ignition::math::equal(0.0f, clr.g));
  EXPECT_TRUE(ignition::math::equal(0.9064f, clr.b, 1e-3f));
  EXPECT_TRUE(ignition::math::equal(0.04f, clr.a));

  EXPECT_TRUE(clr.YUV() ==
      ignition::math::Vector3d(0.104985, 0.95227, 0.429305));

  // Note: ideally the addition should result in (1.0, 0.3, 0.9, 1.0);
  clr = common::Color(1.0f, 0.0f, 0.5f, 1.0f)
      + common::Color(0.1f, 0.3f, 0.4f, 1.0f);
  EXPECT_TRUE(ignition::math::equal(0.00431373f, clr.r));
  EXPECT_TRUE(ignition::math::equal(0.3f, clr.g));
  EXPECT_TRUE(ignition::math::equal(0.9f, clr.b));
  EXPECT_TRUE(ignition::math::equal(0.00784314f, clr.a));

  clr = common::Color(1.0f, 0.0f, 0.5f, 1.0f)
      - common::Color(0.1f, 0.3f, 0.4f, 1.0f);
  EXPECT_TRUE(ignition::math::equal(0.9f, clr.r));
  EXPECT_TRUE(ignition::math::equal(0.0f, clr.g));
  EXPECT_TRUE(ignition::math::equal(0.1f, clr.b));
  EXPECT_TRUE(ignition::math::equal(0.0f, clr.a));

  clr = common::Color(0.5f, 0.2f, 0.4f, 0.6f) / 2.0;
  EXPECT_TRUE(ignition::math::equal(0.25f, clr.r));
  EXPECT_TRUE(ignition::math::equal(0.1f, clr.g));
  EXPECT_TRUE(ignition::math::equal(0.2f, clr.b));
  EXPECT_TRUE(ignition::math::equal(0.3f, clr.a));

  // test color rgb value clamping
  clr = common::Color(ignition::math::NAN_F, 51, 256, -1.0);
  EXPECT_TRUE(ignition::math::equal(0.0f, clr.r));
  EXPECT_TRUE(ignition::math::equal(0.2f, clr.g));
  EXPECT_TRUE(ignition::math::equal(1.0f, clr.b));
  EXPECT_TRUE(ignition::math::equal(0.0f, clr.a));
}

TEST_F(Color, Ign)
{
  auto clr = common::Color(0.1f, 0.2f, 0.3f, 0.4f).Ign();
  EXPECT_FLOAT_EQ(0.1f, clr.R());
  EXPECT_FLOAT_EQ(0.2f, clr.G());
  EXPECT_FLOAT_EQ(0.3f, clr.B());
  EXPECT_FLOAT_EQ(0.4f, clr.A());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
