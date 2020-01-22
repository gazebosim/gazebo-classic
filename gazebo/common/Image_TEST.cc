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
#include <ignition/math/Color.hh>

#include "gazebo/common/Image.hh"
#include "test/util.hh"

using namespace gazebo;

class ImageTest : public gazebo::testing::AutoLogFixture { };

TEST_F(ImageTest, Image)
{
  common::Image img;
  EXPECT_EQ(-1, img.Load("/file/shouldn/never/exist.png"));
  EXPECT_EQ(0, img.Load("file://media/materials/textures/wood.jpg"));
  EXPECT_EQ(static_cast<unsigned int>(496), img.GetWidth());
  EXPECT_EQ(static_cast<unsigned int>(329), img.GetHeight());
  EXPECT_EQ(static_cast<unsigned int>(24), img.GetBPP());
  EXPECT_TRUE(img.Pixel(10, 10) ==
      ignition::math::Color(0.133333, 0.376471, 0.654902, 1));
  EXPECT_TRUE(img.AvgColor() ==
      ignition::math::Color(0.260456, 0.506047, 0.758062, 1));
  EXPECT_TRUE(img.MaxColor() ==
      ignition::math::Color(0.807843, 0.909804, 0.964706, 1));
  EXPECT_TRUE(img.Valid());
  EXPECT_TRUE(img.GetFilename().find("materials/textures/wood.jpg") !=
      std::string::npos);

  unsigned char *data = nullptr;
  unsigned int size = 0;
  img.GetData(&data, size);
  EXPECT_EQ(static_cast<unsigned int>(489552), size);

  img.SetFromData(data, img.GetWidth(), img.GetHeight(),
                  common::Image::RGB_INT8);
}

/////////////////////////////////////////////////
TEST_F(ImageTest, ConvertPixelFormat)
{
  using Image = gazebo::common::Image;
  EXPECT_EQ(Image::PixelFormat::UNKNOWN_PIXEL_FORMAT,
     Image::ConvertPixelFormat("fake"));
  EXPECT_EQ(Image::PixelFormat::UNKNOWN_PIXEL_FORMAT,
     Image::ConvertPixelFormat("unknown"));
  EXPECT_EQ(Image::PixelFormat::UNKNOWN_PIXEL_FORMAT,
     Image::ConvertPixelFormat("UNKNOWN_PIXEL_FORMAT"));
  EXPECT_EQ(Image::PixelFormat::L_INT8,
     Image::ConvertPixelFormat("L_INT8"));
  EXPECT_EQ(Image::PixelFormat::L_INT16,
     Image::ConvertPixelFormat("L_INT16"));
  EXPECT_EQ(Image::PixelFormat::RGB_INT8,
     Image::ConvertPixelFormat("RGB_INT8"));
  EXPECT_EQ(Image::PixelFormat::RGBA_INT8,
     Image::ConvertPixelFormat("RGBA_INT8"));
  EXPECT_EQ(Image::PixelFormat::RGB_INT16,
     Image::ConvertPixelFormat("RGB_INT16"));
  EXPECT_EQ(Image::PixelFormat::RGB_INT32,
     Image::ConvertPixelFormat("RGB_INT32"));
  EXPECT_EQ(Image::PixelFormat::BGR_INT8,
     Image::ConvertPixelFormat("BGR_INT8"));
  EXPECT_EQ(Image::PixelFormat::BGRA_INT8,
     Image::ConvertPixelFormat("BGRA_INT8"));
  EXPECT_EQ(Image::PixelFormat::BGR_INT16,
     Image::ConvertPixelFormat("BGR_INT16"));
  EXPECT_EQ(Image::PixelFormat::BGR_INT32,
     Image::ConvertPixelFormat("BGR_INT32"));
  EXPECT_EQ(Image::PixelFormat::R_FLOAT16,
     Image::ConvertPixelFormat("R_FLOAT16"));
  EXPECT_EQ(Image::PixelFormat::R_FLOAT32,
     Image::ConvertPixelFormat("R_FLOAT32"));
  EXPECT_EQ(Image::PixelFormat::RGB_FLOAT16,
     Image::ConvertPixelFormat("RGB_FLOAT16"));
  EXPECT_EQ(Image::PixelFormat::RGB_FLOAT32,
     Image::ConvertPixelFormat("RGB_FLOAT32"));
  EXPECT_EQ(Image::PixelFormat::BAYER_RGGB8,
     Image::ConvertPixelFormat("BAYER_RGGB8"));
  EXPECT_EQ(Image::PixelFormat::BAYER_GBRG8,
     Image::ConvertPixelFormat("BAYER_GBRG8"));
  EXPECT_EQ(Image::PixelFormat::BAYER_GRBG8,
     Image::ConvertPixelFormat("BAYER_GRBG8"));
  EXPECT_EQ(Image::PixelFormat::BAYER_BGGR8,
     Image::ConvertPixelFormat("BAYER_BGGR8"));
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
