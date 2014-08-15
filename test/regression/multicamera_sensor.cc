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

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

#include "ServerFixture.hh"
#include "scans_cmp.h"

using namespace gazebo;

namespace gazebo
{
namespace sensors
{
  // can we add this to SensorTypes.hh without breaking ABI?
  typedef boost::shared_ptr<MultiCameraSensor> MultiCameraSensorPtr;
}
}

class MultiCameraSensor : public ServerFixture
{
};

unsigned char* img0Left = NULL;
unsigned char* imgt = NULL;
unsigned char* img1Left = NULL;
unsigned char* img2Left = NULL;
unsigned char* img0Right = NULL;
unsigned char* img1Right = NULL;
unsigned char* img2Right = NULL;
int imageCount0Left = 0;
int imageCountt = 0;
int imageCount1Left = 0;
int imageCount2Left = 0;
int imageCount0Right = 0;
int imageCount1Right = 0;
int imageCount2Right = 0;

/////////////////////////////////////////////////
void OnNewFrameTest(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
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
  std::string cameraUnrotated = "multicamera_sensor_unrotated";
  std::string cameraTranslated = "multicamera_sensor_translated";
  std::string cameraRotated1 = "multicamera_sensor_rotated1";
  std::string cameraRotated2 = "multicamera_sensor_rotated2";

  sensors::SensorPtr sensor = sensors::get_sensor(cameraUnrotated);
  sensors::MultiCameraSensorPtr camSensorUnrotated =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraTranslated);
  sensors::CameraSensorPtr camSensorTranslated =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

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
  imageCount0Left  = 0;
  imageCountt  = 0;
  imageCount1Left  = 0;
  imageCount2Left  = 0;
  imageCount0Right = 0;
  imageCount1Right = 0;
  imageCount2Right = 0;
  img0Left = new unsigned char[width * height * depth];
  imgt = new unsigned char[width * height * depth];
  img1Left = new unsigned char[width * height * depth];
  img2Left = new unsigned char[width * height * depth];
  img0Right = new unsigned char[width * height * depth];
  img1Right = new unsigned char[width * height * depth];
  img2Right = new unsigned char[width * height * depth];

  {
    // connect to camera image updates
    event::ConnectionPtr c0Left =
      camSensorUnrotated->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount0Left, img0Left,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr ct =
      camSensorTranslated->GetCamera()->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCountt, imgt,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c1Left =
      camSensorRotated1->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount1Left, img1Left,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c2Left =
      camSensorRotated2->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount2Left, img2Left,
            _1, _2, _3, _4, _5));

    event::ConnectionPtr c0Right =
      camSensorUnrotated->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount0Right, img0Right,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c1Right =
      camSensorRotated1->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount1Right, img1Right,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c2Right =
      camSensorRotated2->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount2Right, img2Right,
            _1, _2, _3, _4, _5));

    // activate camera
    camSensorUnrotated->SetActive(true);
    camSensorTranslated->SetActive(true);
    camSensorRotated1->SetActive(true);
    camSensorRotated2->SetActive(true);

    // Get at least 10 images from each camera
    int waitCount = 0;
    while (imageCount0Left < 10 || imageCountt < 10 ||
           imageCount1Left < 10 || imageCount2Left < 10 ||
           imageCount0Right < 10 ||
           imageCount1Right < 10 || imageCount2Right < 10)
    {
      // wait at most 10 seconds sim time.
      if (++waitCount >= 1000)
        gzerr << "Err [" << imageCount0Left
              << "/10, " << imageCountt
              << "/10, " << imageCount1Left
              << "/10, " << imageCount2Left
              << "/10, " << imageCount0Right
              << "/10, " << imageCount1Right
              << "/10, " << imageCount2Right
              << "/10] images received from cameras\n";
      EXPECT_LT(waitCount, 1000);

      common::Time::MSleep(10);
    }

    // compare unrotated left against translated left, both should be
    // seeing the green block, if not, pose translation in <camera> tag
    // is broken.
    // Note, right camera of translated camera (imgtRight) should also
    // see green block.
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare left images
      this->ImageCompare(img0Left, imgt, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated left against translated left, both should be
    // seeing the green block, if not, pose translation in <camera> tag
    // is broken.
    // Note, right camera of translated camera (imgtRight) should also
    // see green block.
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = -1.0;

      // compare right of translated camera to
      // right of unrotated/untranslated camera images
      // the result should differ as former sees green block and latter
      // sees red block.
      this->ImageCompare(img0Right, imgt, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // use below to construct test for rotated2 left camera offset
      // math::Quaternion a(1.2, 1.3, 1.4);
      // gzerr << "test: " << a.RotateVector(math::Vector3(0, 1, 0)) << "\n";

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_GT(diffSum, 100u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_GT(fabs(diffAvg)/255., 0.0549);
    }

    // compare unrotated against rotated1
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare left images
      this->ImageCompare(img0Left, img1Left, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated against rotated1
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;
      // compare right images
      this->ImageCompare(img0Right, img1Right, width, height, depth,
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

      // compare left images
      this->ImageCompare(img0Left, img2Left, width, height, depth,
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

      // compare right images
      this->ImageCompare(img0Right, img2Right, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // activate camera
    camSensorUnrotated->SetActive(false);
    camSensorTranslated->SetActive(false);
    camSensorRotated1->SetActive(false);
    camSensorRotated2->SetActive(false);

    // disconnect callbacks
    camSensorUnrotated->GetCamera(0)->DisconnectNewImageFrame(c0Left);
    camSensorTranslated->GetCamera()->DisconnectNewImageFrame(ct);
    camSensorRotated1->GetCamera(0)->DisconnectNewImageFrame(c1Left);
    camSensorRotated2->GetCamera(0)->DisconnectNewImageFrame(c2Left);
    camSensorUnrotated->GetCamera(1)->DisconnectNewImageFrame(c0Right);
    camSensorRotated1->GetCamera(1)->DisconnectNewImageFrame(c1Right);
    camSensorRotated2->GetCamera(1)->DisconnectNewImageFrame(c2Right);
  }

  // cleanup
  delete[] img0Left;
  delete[] imgt;
  delete[] img1Left;
  delete[] img2Left;
  delete[] img0Right;
  delete[] img1Right;
  delete[] img2Right;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
