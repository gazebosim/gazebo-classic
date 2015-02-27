/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "ServerFixture.hh"
#include "scans_cmp.h"

using namespace gazebo;
class CameraSensor : public ServerFixture
{
};

boost::mutex mutex;

unsigned char* img = NULL;
unsigned char* img2 = NULL;
int imageCount = 0;
int imageCount2 = 0;
std::string pixelFormat = "";

/////////////////////////////////////////////////
void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &_format)
{
  boost::mutex::scoped_lock lock(mutex);
  pixelFormat = _format;
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, CheckThrottle)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn sensors of various sizes to test speed
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;  // 106 fps
  double updateRate = 10;
  math::Pose setPose, testPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  common::Timer timer;
  timer.Start();

  // time how long it takes to get 50 images @ 10Hz
  int total_images = 50;

  while (imageCount < total_images)
    common::Time::MSleep(10);
  common::Time dt = timer.GetElapsed();
  double rate = static_cast<double>(total_images)/dt.Double();
  gzdbg << "timer [" << dt.Double() << "] seconds rate [" << rate << "] fps\n";
  EXPECT_GT(rate, 7.0);
  EXPECT_LT(rate, 11.0);
  camSensor->GetCamera()->DisconnectNewImageFrame(c);
  delete img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, UnlimitedTest)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn sensors of various sizes to test speed
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";

  // test resolution, my machine gets about 106 fps
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 0;
  math::Pose setPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  common::Timer timer;
  timer.Start();
  // time how long it takes to get N images
  int total_images = 500;
  while (imageCount < total_images)
    common::Time::MSleep(10);
  common::Time dt = timer.GetElapsed();
  double rate = static_cast<double>(total_images)/dt.Double();
  gzdbg << "timer [" << dt.Double() << "] seconds rate [" << rate << "] fps\n";
  camSensor->GetCamera()->DisconnectNewImageFrame(c);
  EXPECT_GT(rate, 30.0);

  delete img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, MultiSenseHigh)
{
  // This test is disabled because it does not work on machines with
  // limited rendering capabilities.
  return;
/*
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn sensors of various sizes to test speed
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";

  // nominal resolution of multisense
  unsigned int width  = 2048;
  unsigned int height = 1088;
  double updateRate = 25;
  math::Pose setPose, testPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  common::Timer timer;
  timer.Start();
  // time how long it takes to get N images
  int total_images = 500;
  while (imageCount < total_images)
    common::Time::MSleep(10);
  common::Time dt = timer.GetElapsed();
  double rate = static_cast<double>(total_images)/dt.Double();
  gzdbg << "timer [" << dt.Double() << "] seconds rate [" << rate << "] fps\n";
  camSensor->GetCamera()->DisconnectNewImageFrame(c);
  EXPECT_GT(rate, 24.0);
  EXPECT_LT(rate, 25.0);

  delete img;
  */
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, MultiSenseLow)
{
  // This test is disabled because it does not work on machines with
  // limited rendering capabilities.
  return;
/*
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn sensors of various sizes to test speed
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";

  // lower resolution of multisense
  unsigned int width  = 1024;
  unsigned int height = 544;
  double updateRate = 25;
  math::Pose setPose, testPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  common::Timer timer;
  timer.Start();
  // time how long it takes to get N images
  int total_images = 500;
  while (imageCount < total_images)
    common::Time::MSleep(10);
  common::Time dt = timer.GetElapsed();
  double rate = static_cast<double>(total_images)/dt.Double();
  gzdbg << "timer [" << dt.Double() << "] seconds rate [" << rate << "] fps\n";
  camSensor->GetCamera()->DisconnectNewImageFrame(c);
  EXPECT_GT(rate, 24.0);
  EXPECT_LT(rate, 25.0);

  delete img;
  Unload();
  */
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, CheckNoise)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Spawn two cameras in the same location, one with noise and one without.
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  std::string modelNameNoisy = "camera_model_noisy";
  std::string cameraNameNoisy = "camera_sensor_noisy";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  double noiseMean = 0.1;
  double noiseStdDev = 0.01;
  math::Pose setPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  SpawnCamera(modelNameNoisy, cameraNameNoisy, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate,
      "gaussian", noiseMean, noiseStdDev);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  sensor = sensors::get_sensor(cameraNameNoisy);
  sensors::CameraSensorPtr camSensorNoisy =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  event::ConnectionPtr c2 =
    camSensorNoisy->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount2, img2,
          _1, _2, _3, _4, _5));

  // Get some images
  while (imageCount < 10 || imageCount2 < 10)
    common::Time::MSleep(10);

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;
  this->ImageCompare(img, img2, width, height, 3,
                     diffMax, diffSum, diffAvg);
  // We expect that there will be some non-zero difference between the two
  // images.
  EXPECT_NE(diffSum, 0u);
  // We expect that the average difference will be well within 3-sigma.
  EXPECT_NEAR(diffAvg/255., noiseMean, 3*noiseStdDev);
  delete[] img;
  delete[] img2;
}


/////////////////////////////////////////////////
TEST_F(CameraSensor, CheckDistortion)
{
  Load("worlds/empty.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Spawn two cameras in the same location, one with noise and one without.
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  std::string modelNameDistorted = "camera_model_distorted";
  std::string cameraNameDistorted = "camera_sensor_distorted";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;

  math::Pose setPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  // spawn a camera with barrel distortion
  SpawnCamera(modelNameDistorted, cameraNameDistorted, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate,
      "", 0, 0, true, -0.25349, 0.11868, 0.0, -0.00028, 0.00005, 0.5, 0.5);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  sensor = sensors::get_sensor(cameraNameDistorted);
  sensors::CameraSensorPtr camSensorDistorted =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  event::ConnectionPtr c2 =
    camSensorDistorted->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount2, img2,
          _1, _2, _3, _4, _5));

  // Get some images
  while (imageCount < 10 || imageCount2 < 10)
    common::Time::MSleep(10);

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;
  this->ImageCompare(img, img2, width, height, 3,
                     diffMax, diffSum, diffAvg);

  // We expect that there will be some non-zero difference between the two
  // images.
  EXPECT_NE(diffSum, 0u);

  // Compare colors. Distorted image should have more darker pixels than the
  // original as the ground plane has been warped to occupy more of the image.
  unsigned int colorSum = 0;
  unsigned int colorSum2 = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3)];
      unsigned int g = img[(y*width*3)+1];
      unsigned int b = img[(y*width*3)+2];
      colorSum += r + g + b;
      unsigned int r2 = img2[(y*width*3)];
      unsigned int g2 = img2[(y*width*3)+1];
      unsigned int b2 = img2[(y*width*3)+2];
      colorSum2 += r2 + g2 + b2;
    }
  }
  EXPECT_GT(colorSum, colorSum2);

  // We expect that there will be some non-zero difference between the two
  // images.
  EXPECT_NE(diffSum, 0u);
  delete[] img;
  delete[] img2;
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


// Place two cameras at some distance apart and a box in between
// them. Verify they generate different images.
TEST_F(CameraSensor, CompareSideBySideCamera)
{
  Load("worlds/empty.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Spawn two cameras at 2m apart.
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  std::string modelName2 = "camera_model2";
  std::string cameraName2 = "camera_sensor2";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;

  math::Pose testPose(
      math::Vector3(0, 0, 0.5), math::Quaternion(0, 0, 0));
  math::Pose testPose2(
      math::Vector3(0, 2, 0.5), math::Quaternion(0, 0, 0));
  SpawnCamera(modelName, cameraName, testPose.pos,
      testPose.rot.GetAsEuler(), width, height, updateRate);
  SpawnCamera(modelName2, cameraName2, testPose2.pos,
      testPose.rot.GetAsEuler(), width, height, updateRate);

  // Spawn a box in front of the cameras
  SpawnBox("test_box", math::Vector3(1, 1, 1),
      math::Vector3(4, 1, 0.5), math::Vector3(0, 0, 0));

  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  sensor = sensors::get_sensor(cameraName2);
  sensors::CameraSensorPtr camSensor2 =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height*3];
  unsigned char *prevImg = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  unsigned char *prevImg2 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));
  event::ConnectionPtr c2 =
    camSensor2->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount2, img2,
          _1, _2, _3, _4, _5));

  while (imageCount < 10 || imageCount2 < 10)
    common::Time::MSleep(10);

  memcpy(prevImg, img, width * height * 3);
  memcpy(prevImg2, img2, width * height * 3);

  for (int i = 0; i < 10; ++i)
  {
    imageCount = 0;
    imageCount2 = 0;

    // Get some images
    while (imageCount < 1 || imageCount2 < 1)
      common::Time::MSleep(10);

    unsigned int diffMax12 = 0;
    unsigned int diffSum12 = 0;
    unsigned int diffSum = 0;
    unsigned int diffSum2 = 0;
    double diffAvg12 = 0.0;
    {
      unsigned int diffMax = 0;
      double diffAvg = 0.0;
      unsigned int diffMax2 = 0;
      double diffAvg2 = 0.0;

      boost::mutex::scoped_lock lock(mutex);
      this->ImageCompare(img, prevImg, width, height, 3,
                         diffMax, diffSum, diffAvg);
      this->ImageCompare(prevImg2, prevImg2, width, height, 3,
                         diffMax2, diffSum2, diffAvg2);
      this->ImageCompare(img, img2, width, height, 3,
                         diffMax12, diffSum12, diffAvg12);
      memcpy(prevImg, img, width * height * 3);
      memcpy(prevImg2, img2, width * height * 3);
    }

    // Images from the same camera should be identical
    EXPECT_EQ(diffSum, 0u);
    EXPECT_EQ(diffSum2, 0u);

    // We expect that there will some noticeable difference
    // between the two different camera images.
    EXPECT_NE(diffSum12, 1000000u);
    EXPECT_GT(diffAvg12, 0.0);
    EXPECT_GT(diffMax12, 0.0);

    common::Time::MSleep(100);
  }
  delete[] img;
  delete[] img2;
  delete[] prevImg;
  delete[] prevImg2;
}
