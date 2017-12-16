/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <functional>

#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/rendering/Camera.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class WideAngleCameraSensor : public ServerFixture
{
};

std::mutex mutex;

unsigned char *img = nullptr;
int imageCount = 0;
std::string pixelFormat = "";

/////////////////////////////////////////////////
void OnNewCameraFrame(int *_imageCounter, unsigned char *_imageDest,
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
TEST_F(WideAngleCameraSensor, Background)
{
#if not defined(__APPLE__)
  Load("worlds/usercamera_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run wide angle camera test\n";
    return;
  }

  // Spawn a wide angle camera
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose = ignition::math::Pose3d::Zero;
  SpawnWideAngleCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate, 6.0);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::WideAngleCameraSensorPtr camSensor =
      std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);

  imageCount = 0;
  img = new unsigned char[width * height * 3];
  event::ConnectionPtr c =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // Get some images
  int sleep = 0;
  int maxSleep = 30;
  while (imageCount < 10 && sleep < maxSleep)
  {
    common::Time::MSleep(50);
    sleep++;
  }

  // check image bg color. It should be black instead of the default grey
  unsigned int rSum = 0;
  unsigned int gSum = 0;
  unsigned int bSum = 0;
  for (unsigned int i = 0; i < height*width*3; i+=3)
  {
    unsigned int r = img[i];
    unsigned int g = img[i+1];
    unsigned int b = img[i+2];
    rSum += r;
    gSum += g;
    bSum += b;
  }

  EXPECT_DOUBLE_EQ(rSum, 0.0);
  EXPECT_DOUBLE_EQ(gSum, 0.0);
  EXPECT_DOUBLE_EQ(bSum, 0.0);

  delete [] img;
#endif
}

/////////////////////////////////////////////////
TEST_F(WideAngleCameraSensor, Projection)
{
#if not defined(__APPLE__)
  Load("worlds/usercamera_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run wide angle camera test\n";
    return;
  }

  // Spawn a wide angle camera
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose = ignition::math::Pose3d::Zero;
  SpawnWideAngleCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate, 6.0);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::WideAngleCameraSensorPtr camSensor =
      std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor);

  rendering::WideAngleCameraPtr camera =
      boost::dynamic_pointer_cast<rendering::WideAngleCamera>(
      camSensor->Camera());
  ASSERT_NE(camera, nullptr);

  // point directly in front of camera
  auto worldPoint = ignition::math::Vector3d::UnitX;
  auto screenPt = camera->Project3d(worldPoint);
  EXPECT_FLOAT_EQ(camera->ViewportWidth() * 0.5, screenPt.X());
  EXPECT_FLOAT_EQ(camera->ViewportHeight() * 0.5, screenPt.Y());
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point behind camera
  worldPoint = -ignition::math::Vector3d::UnitX;
  screenPt = camera->Project3d(worldPoint);
  // z is distance of point from image center
  // in this case it'll be outside of image so greater than 1.0
  EXPECT_GT(screenPt.Z(), 1.0);

  // point at right side of camera image
  worldPoint = ignition::math::Vector3d(1, -0.5, 0.0);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_GT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_FLOAT_EQ(camera->ViewportHeight() * 0.5, screenPt.Y());
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at left side of camera image
  worldPoint = ignition::math::Vector3d(1, 0.5, 0.0);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_LT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_FLOAT_EQ(camera->ViewportHeight() * 0.5, screenPt.Y());
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at top half of camera image
  worldPoint = ignition::math::Vector3d(1, 0.0, 0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_FLOAT_EQ(camera->ViewportWidth() * 0.5, screenPt.X());
  EXPECT_LT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at bottom half of camera image
  worldPoint = ignition::math::Vector3d(1, 0.0, -0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_FLOAT_EQ(camera->ViewportWidth() * 0.5, screenPt.X());
  EXPECT_GT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at top left quadrant of camera image
  worldPoint = ignition::math::Vector3d(1, 0.5, 0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_LT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_LT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at top right quadrant of camera image
  worldPoint = ignition::math::Vector3d(1, -0.5, 0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_GT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_LT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at bottom left quadrant of camera image
  worldPoint = ignition::math::Vector3d(1, 0.5, -0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_LT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_GT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);

  // point at bottom right quadrant of camera image
  worldPoint = ignition::math::Vector3d(1, -0.5, -0.5);
  screenPt = camera->Project3d(worldPoint);
  EXPECT_GT(screenPt.X(), camera->ViewportWidth() * 0.5);
  EXPECT_GT(screenPt.Y(), camera->ViewportHeight() * 0.5);
  EXPECT_GT(screenPt.Z(), 0.0);
  EXPECT_LT(screenPt.Z(), 1.0);
#endif
}
