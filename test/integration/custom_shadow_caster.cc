/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class CustomShadowCasterTest : public ServerFixture
{
};

std::mutex mutex;

unsigned char* img = NULL;
int imageCount = 0;
std::string pixelFormat = "";

/////////////////////////////////////////////////
void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &_format)
{
  std::lock_guard<std::mutex> lock(mutex);
  pixelFormat = _format;
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
// \brief Use the "Gazebo/shadow_caster" material for shadow
/// casting. Shadows will be enabled from the material.
TEST_F(CustomShadowCasterTest, DefaultShadowCaster)
{
  Load("worlds/default_shadow_caster.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  sensors::CameraSensorPtr sensor =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_normal"));
  EXPECT_TRUE(sensor != nullptr);

  unsigned int width  = 320;
  unsigned int height = 240;

  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    sensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Get some images
  while (imageCount < 10)
  {
    common::Time::MSleep(10);
  }

  unsigned int colorSum = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      colorSum += r + g + b;
    }
  }

  EXPECT_GT(18000u, colorSum);
  EXPECT_LT(14000u, colorSum);
}

/////////////////////////////////////////////////
/// \brief Use the "Gazebo/shadow_caster_ignore_heightmap" material for shadow
/// casting. Shadows will be disabled from the material.
TEST_F(CustomShadowCasterTest, CustomShadowCaster)
{
  Load("worlds/custom_shadow_caster.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  sensors::CameraSensorPtr sensor =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_normal"));
  EXPECT_TRUE(sensor != nullptr);

  unsigned int width  = 320;
  unsigned int height = 240;

  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    sensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  // Get some images
  while (imageCount < 10)
  {
    common::Time::MSleep(10);
  }

  unsigned int colorSum = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      colorSum += r + g + b;
    }
  }

  EXPECT_GT(31000u, colorSum);
  EXPECT_LT(27000u, colorSum);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
