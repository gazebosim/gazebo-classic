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

#include "gazebo/common/Image.hh"

using namespace gazebo;

TEST(ImageTest, Image)
{
  common::Image img;
  EXPECT_EQ(-1, img.Load("/file/shouldn/never/exist.png"));
  EXPECT_EQ(0, img.Load("file://media/materials/textures/wood.jpg"));
  EXPECT_EQ(static_cast<unsigned int>(496), img.GetWidth());
  EXPECT_EQ(static_cast<unsigned int>(329), img.GetHeight());
  EXPECT_EQ(static_cast<unsigned int>(24), img.GetBPP());
  EXPECT_TRUE(img.GetPixel(10, 10) ==
      common::Color(0.133333, 0.376471, 0.654902, 1));
  EXPECT_TRUE(img.GetAvgColor() ==
      common::Color(0.260456, 0.506047, 0.758062, 1));
  EXPECT_TRUE(img.GetMaxColor() ==
      common::Color(0.807843, 0.909804, 0.964706, 1));
  EXPECT_TRUE(img.Valid());
  EXPECT_TRUE(img.GetFilename().find("materials/textures/wood.jpg") !=
      std::string::npos);

  unsigned char *data = NULL;
  unsigned int size = 0;
  img.GetData(&data, size);
  EXPECT_EQ(static_cast<unsigned int>(489552), size);

  img.SetFromData(data, img.GetWidth(), img.GetHeight(),
                  common::Image::RGB_INT8);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
