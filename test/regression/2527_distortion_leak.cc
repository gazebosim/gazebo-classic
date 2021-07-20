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

unsigned char* img = NULL;
unsigned char* img3 = NULL;
unsigned char* img4 = NULL;
int imageCount = 0;
int imageCount3 = 0;
int imageCount4 = 0;
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
/// \brief Test for issue #2527. Test is the same as CheckDistortion
/// "camera_sensor.cc" but loads the cameras from a world file instead,
//  of dynamically adding cameras.
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

  sensors::CameraSensorPtr camSensorUndistorted =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_sensor_undistorted"));
  EXPECT_TRUE(camSensorUndistorted != nullptr);

  sensors::CameraSensorPtr camSensorBarrel =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_sensor_barrel"));
  EXPECT_TRUE(camSensorBarrel != nullptr);

  sensors::CameraSensorPtr camSensorPincushion =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
      sensors::get_sensor("camera_sensor_pincushion"));
  EXPECT_TRUE(camSensorPincushion != nullptr);

  unsigned int width  = 320;
  unsigned int height = 240;

  imageCount = 0;
  imageCount3 = 0;
  imageCount4 = 0;
  img = new unsigned char[width * height*3];
  img3 = new unsigned char[width * height*3];
  img4 = new unsigned char[width * height*3];

  event::ConnectionPtr c =
    camSensorUndistorted->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c3 =
    camSensorBarrel->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount3, img3,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c4 =
    camSensorPincushion->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount4, img4,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  while (imageCount < 10 || imageCount3 < 10 || imageCount4 < 10)
  {
    common::Time::MSleep(10);
  }

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;

  // We expect that there will be some non-zero difference between the images,
  // except for the 0.0 distortion camera, which should return a completely
  // identical camera to the one with no distortion tag in the SDF.

  this->ImageCompare(img, img3, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(0u, diffSum);

  this->ImageCompare(img, img4, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(0u, diffSum);

  this->ImageCompare(img3, img4, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(0u, diffSum);

  // Compare colors. Barrel distorted image should have more darker pixels than
  // the original as the ground plane has been warped to occupy more of the
  // image. The same should be true for pincushion distortion, because the
  // ground plane is still distorted to be larger - just different parts
  // of the image are distorted.
  unsigned int colorSum = 0;
  unsigned int colorSum3 = 0;
  unsigned int colorSum4 = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      colorSum += r + g + b;
      unsigned int r3 = img3[(y*width*3) + x];
      unsigned int g3 = img3[(y*width*3) + x + 1];
      unsigned int b3 = img3[(y*width*3) + x + 2];
      colorSum3 += r3 + g3 + b3;
      unsigned int r4 = img4[(y*width*3) + x];
      unsigned int g4 = img4[(y*width*3) + x + 1];
      unsigned int b4 = img4[(y*width*3) + x + 2];
      colorSum4 += r4 + g4 + b4;
    }
  }
  EXPECT_GT(colorSum, colorSum3);
  EXPECT_GT(colorSum, colorSum4);

  delete[] img;
  delete[] img3;
  delete[] img4;
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
