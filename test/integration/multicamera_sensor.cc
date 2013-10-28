/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

#include "ServerFixture.hh"
#include "scans_cmp.h"

using namespace gazebo;
class MultiCameraSensor : public ServerFixture
{
};

unsigned char* img = NULL;
unsigned char* img1 = NULL;
unsigned char* img2 = NULL;
int imageCount = 0;
int imageCount1 = 0;
int imageCount2 = 0;

void OnNewFrameTest(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}


TEST_F(MultiCameraSensor, CameraRotationTest)
{
  Load("worlds/camera_rotation_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // get two cameras, one with rotation and one without.
  std::string modelUnrotated = "cam_x_rot_test_unrotated_cameras_1";
  std::string cameraUnrotated = "multicamera_sensor";
  std::string modelRotated1 = "cam_x_rot_test_rotated_cameras_1";
  std::string cameraRotated1 = "multicamera_sensor";
  std::string modelRotated2 = "cam_x_rot_test_rotated_cameras_2";
  std::string cameraRotated2 = "multicamera_sensor";

  sensors::SensorPtr sensor = sensors::get_sensor(cameraUnrotated);
  sensors::MultiCameraSensorPtr camSensorUnrotated =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated1);
  sensors::MultiCameraSensorPtr camSensorRotated1 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated2);
  sensors::MultiCameraSensorPtr camSensorRotated2 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  unsigned int width  = 1024;
  unsigned int height = 544;
  unsigned int depth = 3;

  // initialize global variables
  imageCount = 0;
  imageCount1 = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height * depth];
  img1 = new unsigned char[width * height * depth];
  img2 = new unsigned char[width * height * depth];

  for (unsigned int i = 0; i < 2; ++i)
  {
    // connect to camera image updates
    event::ConnectionPtr c =
      camSensorUnrotated->GetCamera(i)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount, img,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c1 =
      camSensorRotated1->GetCamera(i)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount1, img1,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c2 =
      camSensorRotated2->GetCamera(i)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount2, img2,
            _1, _2, _3, _4, _5));

    // activate camera
    camSensorUnrotated->SetActive(true);
    camSensorRotated1->SetActive(true);
    camSensorRotated2->SetActive(true);

    // Get at least 10 images from each camera
    int waitCount = 0;
    while (imageCount < 10 || imageCount1 < 10 || imageCount2 < 10)
    {
      // wait at most 10 seconds sim time.
      if (++waitCount >= 1000)
        gzerr << "[" << imageCount
              << "/10, " << imageCount1
              << "/10, " << imageCount2
              << "/10] images received from cameras\n";
      EXPECT_LT(waitCount, 1000);

      common::Time::MSleep(10);
    }

    // compare unrotated against rotated1
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;
      this->ImageCompare(img, img1, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated against rotated2
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;
      this->ImageCompare(img, img2, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // activate camera
    camSensorUnrotated->SetActive(false);
    camSensorRotated1->SetActive(false);
    camSensorRotated2->SetActive(false);

    // disconnect callbacks
    camSensorUnrotated->GetCamera(i)->DisconnectNewImageFrame(c);
    camSensorRotated1->GetCamera(i)->DisconnectNewImageFrame(c1);
    camSensorRotated2->GetCamera(i)->DisconnectNewImageFrame(c2);
  }

  // cleanup
  delete[] img;
  delete[] img1;
  delete[] img2;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
