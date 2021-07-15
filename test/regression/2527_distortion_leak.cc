/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <mutex>

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue2527Test : public ServerFixture
{
};

std::mutex mutex;

unsigned char* imgNormal = NULL;
unsigned char* imgBarrel = NULL;
unsigned char* imgPincushion = NULL;
int imageCountNormal = 0;
int imageCountBarrel = 0;
int imageCountPincushion = 0;
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
// \brief Test for issue #1208
TEST_F(Issue2527Test, Distortion)
{
  Load("worlds/issue_2527_camera_distortion.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  sensors::CameraSensorPtr sensorNormal = 
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_normal"));
  EXPECT_TRUE(sensorNormal != nullptr);

  sensors::CameraSensorPtr sensorBarrel = 
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_barrel"));
  EXPECT_TRUE(sensorBarrel != nullptr);

  sensors::CameraSensorPtr sensorPincushion = 
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_pincushion"));
  EXPECT_TRUE(sensorPincushion != nullptr);

  unsigned int width  = 320;
  unsigned int height = 240;

  imageCountNormal = 0;
  imageCountBarrel = 0;
  imageCountPincushion = 0;
  imgNormal = new unsigned char[width * height*3];
  imgBarrel = new unsigned char[width * height*3];
  imgPincushion = new unsigned char[width * height*3];

  event::ConnectionPtr c2 =
    sensorNormal->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCountNormal, imgNormal,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c3 =
    sensorBarrel->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCountBarrel, imgBarrel,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c4 =
    sensorPincushion->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCountPincushion, imgPincushion,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  while (imageCountNormal < 10 || imageCountBarrel < 10 || imageCountPincushion < 10)
  {
    common::Time::MSleep(10);
  }

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;

  this->ImageCompare(imgNormal, imgBarrel, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  this->ImageCompare(imgNormal, imgPincushion, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  this->ImageCompare(imgBarrel, imgPincushion, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  // Compare colors. Barrel distorted image should have more darker pixels than
  // the original as the ground plane has been warped to occupy more of the
  // image. The same should be true for pincushion distortion, because the
  // ground plane is still distorted to be larger - just different parts
  // of the image are distorted.
  unsigned int colorSum2 = 0;
  unsigned int colorSum3 = 0;
  unsigned int colorSum4 = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = imgNormal[(y*width*3) + x];
      unsigned int g = imgNormal[(y*width*3) + x + 1];
      unsigned int b = imgNormal[(y*width*3) + x + 2];
      colorSum2 += r + g + b;
      unsigned int r3 = imgBarrel[(y*width*3) + x];
      unsigned int g3 = imgBarrel[(y*width*3) + x + 1];
      unsigned int b3 = imgBarrel[(y*width*3) + x + 2];
      colorSum3 += r3 + g3 + b3;
      unsigned int r4 = imgPincushion[(y*width*3) + x];
      unsigned int g4 = imgPincushion[(y*width*3) + x + 1];
      unsigned int b4 = imgPincushion[(y*width*3) + x + 2];
      colorSum4 += r4 + g4 + b4;
    }
  }
  EXPECT_GT(colorSum2, colorSum3);
  EXPECT_GT(colorSum2, colorSum4);

  delete[] imgNormal;
  delete[] imgBarrel;
  delete[] imgPincushion;
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
