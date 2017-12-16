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
#include <mutex>
#include <functional>

#include <ignition/math/Rand.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"
#include "scans_cmp.h"

using namespace gazebo;
class CameraSensor : public ServerFixture
{
};

std::mutex mutex;

unsigned char* img = NULL;
unsigned char* img2 = NULL;
unsigned char* img3 = NULL;
unsigned char* img4 = NULL;
int imageCount = 0;
int imageCount2 = 0;
int imageCount3 = 0;
int imageCount4 = 0;
std::string pixelFormat = "";

float *depthImg = nullptr;

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
void OnNewRGBPointCloud(int* _imageCounter, float* _imageDest,
                  const float *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &_format)
{
  std::lock_guard<std::mutex> lock(mutex);
  pixelFormat = _format;
  float f;
  memcpy(_imageDest, _image, _width * _height * sizeof(f) * _depth * 4);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, WorldReset)
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
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose, testPose(
      ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
      camSensor->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  common::Timer timer;
  timer.Start();

  // let the camera render for 2 seconds at 10 Hz
  int total_images = 20;
  while (imageCount < total_images && timer.GetElapsed().Double() < 4)
    common::Time::MSleep(10);
  EXPECT_GE(imageCount, total_images);
  common::Time dt = timer.GetElapsed();
  EXPECT_GT(dt.Double(), 1.0);
  EXPECT_LT(dt.Double(), 3.0);

  // reset the world and verify
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  common::Time simTime = world->SimTime().Double();
  world->Reset();
  EXPECT_TRUE(world->SimTime() == common::Time(0.0) ||
      world->SimTime() < simTime);

  // verify that the camera can continue to render and generate images at
  // the specified rate
  imageCount = 0;
  timer.Reset();
  timer.Start();
  while (imageCount < total_images && timer.GetElapsed().Double() < 4)
    common::Time::MSleep(10);
  dt = timer.GetElapsed();
  EXPECT_GE(imageCount, total_images);
  EXPECT_GT(dt.Double(), 1.0);
  EXPECT_LT(dt.Double(), 3.0);

  c.reset();
  delete [] img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, MultipleCameraSameName)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn first camera sensor
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;  // 106 fps
  double updateRate = 10;
  ignition::math::Pose3d setPose, testPose(
      ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  std::string sensorScopedName =
      "default::" + modelName + "::body::" + cameraName;
  sensors::SensorPtr sensor = sensors::get_sensor(sensorScopedName);
  EXPECT_TRUE(sensor != NULL);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor != NULL);
  rendering::CameraPtr camera = camSensor->Camera();
  EXPECT_TRUE(camera != NULL);

  // spawn second camera sensor with same name but attached to a different model
  std::string modelName2 = modelName + "_2";
  SpawnCamera(modelName2, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  std::string sensorScopedName2 =
      "default::" + modelName2 + "::body::" + cameraName;
  sensors::SensorPtr sensor2 = sensors::get_sensor(sensorScopedName2);
  ASSERT_NE(nullptr, sensor2);
  sensors::CameraSensorPtr camSensor2 =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);
  ASSERT_NE(nullptr, camSensor2);
  rendering::CameraPtr camera2 = camSensor2->Camera();
  EXPECT_TRUE(camera2 != NULL);

  // verify that the sensors and cameras are not the same
  EXPECT_TRUE(camSensor != camSensor2);
  EXPECT_TRUE(camera != camera2);

  // get camera scene and verify camera count
  rendering::ScenePtr scene = camera->GetScene();
  ASSERT_NE(nullptr, scene);
  EXPECT_EQ(scene->CameraCount(), 2u);

  // remove the second camera sensor first and check that it does not remove
  // the first one with the same name
  sensors::remove_sensor(sensorScopedName2);
  int sleep = 0;
  int maxSleep = 10;
  while (sensors::get_sensor(sensorScopedName2) != NULL && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  sensor2 = sensors::get_sensor(sensorScopedName2);
  EXPECT_TRUE(sensor2 == NULL);
  sensor = sensors::get_sensor(sensorScopedName);
  EXPECT_TRUE(sensor != NULL);

  // verify the first camera is still there
  EXPECT_EQ(scene->CameraCount(), 1u);
  EXPECT_TRUE(camera == scene->GetCamera(0));

  std::string renderingCameraName = camera->Name();

  // remove the first camera sensor and there should be no sensors or cameras
  // left
  sensors::remove_sensor(sensorScopedName);
  sleep = 0;
  while (sensors::get_sensor(sensorScopedName) != NULL && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  sensor = sensors::get_sensor(sensorScopedName);
  EXPECT_TRUE(sensor == NULL);
  camera = scene->GetCamera(renderingCameraName);
  EXPECT_TRUE(camera == NULL);
  EXPECT_EQ(scene->CameraCount(), 0u);
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
  ignition::math::Pose3d setPose, testPose(ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c = camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
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
  c.reset();
  delete [] img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, TopicName)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn model with name similar to a nested model
  std::string modelName = "prefix::camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose, testPose(ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  EXPECT_NE(camSensor->Topic().find("prefix/camera_model/body/camera_sensor"),
      std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, FillMsg)
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
  ignition::math::Pose3d setPose(ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  msgs::Sensor msg;
  sensor->FillMsg(msg);

  // Required fields
  EXPECT_EQ(msg.name(), cameraName);
  EXPECT_EQ(msg.parent(), sensor->ParentName());
  EXPECT_EQ(msg.type(), "camera");

  // Optional fields
  ASSERT_TRUE(msg.has_always_on());
  EXPECT_EQ(msg.always_on(), sensor->IsActive());

  ASSERT_TRUE(msg.has_pose());
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()), sensor->Pose());

  ASSERT_TRUE(msg.has_topic());
  EXPECT_EQ(msg.topic(), sensor->Topic());

  ASSERT_TRUE(msg.has_update_rate());
  EXPECT_EQ(msg.update_rate(), sensor->UpdateRate());

  ASSERT_TRUE(msg.has_visualize());
  EXPECT_EQ(msg.visualize(), sensor->Visualize());

  ASSERT_FALSE(msg.has_contact());
  ASSERT_FALSE(msg.has_ray());
  ASSERT_TRUE(msg.has_camera());
  auto cameraMsg = msg.camera();
  auto cam = camSensor->Camera();
  EXPECT_EQ(cameraMsg.horizontal_fov(), cam->HFOV().Radian());
  EXPECT_EQ(cameraMsg.image_size().x(), camSensor->ImageWidth());
  EXPECT_EQ(cameraMsg.image_size().y(), camSensor->ImageHeight());
  EXPECT_EQ(cameraMsg.image_format(), cam->ImageFormat());
  EXPECT_EQ(cameraMsg.near_clip(), cam->NearClip());
  EXPECT_EQ(cameraMsg.far_clip(), cam->FarClip());
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
  ignition::math::Pose3d setPose(ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  common::Timer timer;
  timer.Start();
  // time how long it takes to get N images
  int total_images = 500;
  while (imageCount < total_images)
    common::Time::MSleep(10);
  common::Time dt = timer.GetElapsed();
  double rate = static_cast<double>(total_images)/dt.Double();
  gzdbg << "timer [" << dt.Double() << "] seconds rate [" << rate << "] fps\n";
  c.reset();
  EXPECT_GT(rate, 30.0);

  delete [] img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, MultiSenseHigh)
{
  // This test is disabled because it does not work on machines with
  // limited rendering capabilities.
  return;

//  Load("worlds/empty_test.world");
//
//  // Make sure the render engine is available.
//  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
//      rendering::RenderEngine::NONE)
//  {
//    gzerr << "No rendering engine, unable to run camera test\n";
//    return;
//  }
//
//  // spawn sensors of various sizes to test speed
//  std::string modelName = "camera_model";
//  std::string cameraName = "camera_sensor";
//
//  // nominal resolution of multisense
//  unsigned int width  = 2048;
//  unsigned int height = 1088;
//  double updateRate = 25;
//  math::Pose setPose, testPose(
//      ignition::math::Vector3d(-5, 0, 5),
//      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
//  SpawnCamera(modelName, cameraName, setPose.Pos(),
//      setPose.Rot().Euler(), width, height, updateRate);
//  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
//  sensors::CameraSensorPtr camSensor =
//    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
//  imageCount = 0;
//  img = new unsigned char[width * height*3];
//  event::ConnectionPtr c =
//    camSensor->Camera()->ConnectNewImageFrame(
//        std::bind(&::OnNewCameraFrame, &imageCount, img,
//          _1, _2, _3, _4, _5));
//  common::Timer timer;
//  timer.Start();
//  // time how long it takes to get N images
//  int total_images = 500;
//  while (imageCount < total_images)
//    common::Time::MSleep(10);
//  common::Time dt = timer.GetElapsed();
//  double rate = static_cast<double>(total_images)/dt.Double();
//  gzdbg << "timer [" << dt.Double() << "] seconds rate ["
//        << rate << "] fps\n";
//  c.reset();
//  EXPECT_GT(rate, 24.0);
//  EXPECT_LT(rate, 25.0);
//
//  delete img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, MultiSenseLow)
{
  // This test is disabled because it does not work on machines with
  // limited rendering capabilities.
  return;

//  Load("worlds/empty_test.world");
//
//  // Make sure the render engine is available.
//  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
//      rendering::RenderEngine::NONE)
//  {
//    gzerr << "No rendering engine, unable to run camera test\n";
//    return;
//  }
//
//  // spawn sensors of various sizes to test speed
//  std::string modelName = "camera_model";
//  std::string cameraName = "camera_sensor";
//
//  // lower resolution of multisense
//  unsigned int width  = 1024;
//  unsigned int height = 544;
//  double updateRate = 25;
//  math::Pose setPose, testPose(
//      ignition::math::Vector3d(-5, 0, 5),
//      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
//  SpawnCamera(modelName, cameraName, setPose.Pos(),
//      setPose.Rot().Euler(), width, height, updateRate);
//  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
//  sensors::CameraSensorPtr camSensor =
//    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
//  imageCount = 0;
//  img = new unsigned char[width * height*3];
//  event::ConnectionPtr c =
//    camSensor->Camera()->ConnectNewImageFrame(
//        std::bind(&::OnNewCameraFrame, &imageCount, img,
//          _1, _2, _3, _4, _5));
//  common::Timer timer;
//  timer.Start();
//  // time how long it takes to get N images
//  int total_images = 500;
//  while (imageCount < total_images)
//    common::Time::MSleep(10);
//  common::Time dt = timer.GetElapsed();
//  double rate = static_cast<double>(total_images)/dt.Double();
//  gzdbg << "timer [" << dt.Double() << "] seconds rate ["
//        << rate << "] fps\n";
//  c.reset();
//  EXPECT_GT(rate, 24.0);
//  EXPECT_LT(rate, 25.0);
//
//  delete img;
//  Unload();
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
  ignition::math::Pose3d setPose(ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  SpawnCamera(modelNameNoisy, cameraNameNoisy, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate,
      "gaussian", noiseMean, noiseStdDev);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  sensor = sensors::get_sensor(cameraNameNoisy);
  sensors::CameraSensorPtr camSensorNoisy =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c2 =
    camSensorNoisy->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

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

  // Spawn 4 cameras. One has no distortion.
  // The second has distortion, but all the distortion parameters are set to 0.
  // The third has barrel (negative k1) distortion.
  // The fourth has pincushion (positive k1) distortion.
  std::string modelNameUndistorted = "camera_model_undistorted";
  std::string cameraNameUndistorted = "camera_sensor_undistorted";
  std::string modelNameFlat = "camera_model_flat";
  std::string cameraNameFlat = "camera_sensor_flat";
  std::string modelNameBarrel = "camera_model_barrel";
  std::string cameraNameBarrel = "camera_sensor_barrel";
  std::string modelNamePincushion = "camera_model_pincushion";
  std::string cameraNamePincushion = "camera_sensor_pincushion";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose(
      ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));

  // spawn an undistorted camera
  SpawnCamera(modelNameUndistorted, cameraNameUndistorted, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  // spawn a flat camera
  SpawnCamera(modelNameFlat, cameraNameFlat, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate,
      "", 0, 0, true, 0, 0, 0, 0, 0, 0.5, 0.5);
  // spawn a camera with barrel distortion
  SpawnCamera(modelNameBarrel, cameraNameBarrel, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate,
      "", 0, 0, true, -0.1349, -0.51868, -0.001, 0, 0, 0.5, 0.5);
  // spawn a camera with pincushion distortion
  SpawnCamera(modelNamePincushion, cameraNamePincushion, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate,
      "", 0, 0, true, 0.1349, 0.51868, 0.001, 0, 0, 0.5, 0.5);

  sensors::SensorPtr sensorUndistorted =
    sensors::get_sensor(cameraNameUndistorted);
  sensors::CameraSensorPtr camSensorUndistorted =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorUndistorted);
  sensors::SensorPtr sensorFlat =
    sensors::get_sensor(cameraNameFlat);
  sensors::CameraSensorPtr camSensorFlat =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorFlat);
  sensors::SensorPtr sensorBarrel =
      sensors::get_sensor(cameraNameBarrel);
  sensors::CameraSensorPtr camSensorBarrel =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorBarrel);
  sensors::SensorPtr sensorPincushion =
      sensors::get_sensor(cameraNamePincushion);
  sensors::CameraSensorPtr camSensorPincushion =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorPincushion);

  imageCount = 0;
  imageCount2 = 0;
  imageCount3 = 0;
  imageCount4 = 0;
  img = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  img3 = new unsigned char[width * height*3];
  img4 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensorUndistorted->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c2 =
    camSensorFlat->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
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

  // Get some images
  while (imageCount < 10 || imageCount2 < 10 ||
      imageCount3 < 10 || imageCount4 < 10)
  {
    common::Time::MSleep(10);
  }

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;

  // We expect that there will be some non-zero difference between the images,
  // except for the 0.0 distortion camera, which should return a completely
  // identical camera to the one with no distortion tag in the SDF.

  this->ImageCompare(img, img2, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_EQ(diffSum, 0u);

  this->ImageCompare(img, img3, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  this->ImageCompare(img, img4, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  this->ImageCompare(img3, img4, width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

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
  delete[] img2;
  delete[] img3;
  delete[] img4;
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
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

  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 2, 0.5),
      ignition::math::Quaterniond::Identity);
  SpawnCamera(modelName, cameraName, testPose.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);
  SpawnCamera(modelName2, cameraName2, testPose2.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);

  // Spawn a box in front of the cameras
  SpawnBox("test_box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(4, 1, 0.5), ignition::math::Vector3d::Zero);

  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  sensor = sensors::get_sensor(cameraName2);
  sensors::CameraSensorPtr camSensor2 =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height*3];
  unsigned char *prevImg = new unsigned char[width * height*3];
  img2 = new unsigned char[width * height*3];
  unsigned char *prevImg2 = new unsigned char[width * height*3];
  event::ConnectionPtr c =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c2 =
    camSensor2->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

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

      std::lock_guard<std::mutex> lock(mutex);
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
    // Allow a very small tolerance. There could be a few pixel rgb value
    // changes between frames
    EXPECT_LE(diffSum, 10u);
    EXPECT_LE(diffSum2, 10u);

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

/////////////////////////////////////////////////
TEST_F(CameraSensor, PointCloud)
{
  // world contains a point cloud camera looking at 4 boxes whose faces have
  // different depth in each quadrant of the image
  Load("worlds/pointcloud_camera.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // get point cloud depth camera ssensor
  std::string cameraName = "pointcloud_camera_sensor";
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::DepthCameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
  EXPECT_TRUE(camSensor != nullptr);
  rendering::DepthCameraPtr depthCam = camSensor->DepthCamera();
  EXPECT_TRUE(depthCam != nullptr);

  unsigned int width  = depthCam->ImageWidth();
  unsigned int height = depthCam->ImageHeight();
  EXPECT_GT(width, 0u);
  EXPECT_GT(height, 0u);

  imageCount = 0;
  depthImg = new float[width * height * 4];

  event::ConnectionPtr c = depthCam->ConnectNewRGBPointCloud(
        std::bind(&::OnNewRGBPointCloud, &imageCount, depthImg,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // wait for a few images
  int total_images = 10;
  while (imageCount < total_images)
    common::Time::MSleep(10);

  // get the world
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != nullptr);

  // get the boxes
  physics::ModelPtr boxTR = world->ModelByName("tr_box");
  ASSERT_TRUE(boxTR != nullptr);
  physics::ModelPtr boxTL = world->ModelByName("tl_box");
  ASSERT_TRUE(boxTL != nullptr);
  physics::ModelPtr boxBR = world->ModelByName("br_box");
  ASSERT_TRUE(boxTR != nullptr);
  physics::ModelPtr boxBL = world->ModelByName("bl_box");
  ASSERT_TRUE(boxTL != nullptr);

  // get distance to boxes
  float boxWidth = 1.0;
  float boxHalfWidth = boxWidth * 0.5;
  float distToBoxTR = boxTR->WorldPose().Pos().X() - boxHalfWidth;
  float distToBoxTL = boxTL->WorldPose().Pos().X() - boxHalfWidth;
  float distToBoxBR = boxBR->WorldPose().Pos().X() - boxHalfWidth;
  float distToBoxBL = boxBL->WorldPose().Pos().X() - boxHalfWidth;

  // verify point cloud xyz data for four unit boxes at different distance
  // in front of the point cloud camera.
  // camera uses openni kinect optical frame convention, see comments in
  // issue #2323: x right, y down, z forward
  for (unsigned int i = 0; i < height; ++i)
  {
    // loop through the pixel values
    for (unsigned int j = 0; j < width * 4; j+=4)
    {
      int idx = i * width * 4 + j;
      float x = depthImg[idx];
      float y = depthImg[idx+1];
      float z = depthImg[idx+2];
      // rgb values not valid, see issue #1865
      // int rgb = depthImg[idx+3];

      // left
      if (j < width*4/2)
      {
        // all x values on the left side of camera should be negative and
        EXPECT_LE(x, 0.0);

        // top left
        if (i < height/2)
        {
          EXPECT_LE(y, 0.0);
          EXPECT_NEAR(z, distToBoxTL, 1e-4);
        }
        // bottom left
        else
        {
          EXPECT_GT(y, 0.0);
          EXPECT_NEAR(z, distToBoxBL, 1e-4);
        }
      }
      // right
      else
      {
        // all x values on the right side of camera should be positive
        EXPECT_GT(x, 0.0);

        // top right
        if (i < height/2)
        {
          EXPECT_LE(y, 0.0);
          EXPECT_NEAR(z, distToBoxTR, 1e-4);
        }
        // bottom right
        else
        {
          EXPECT_GT(y, 0.0);
          EXPECT_NEAR(z, distToBoxBR, 1e-4);
        }
      }
      // x and y should be within the width of 2 boxes
      EXPECT_GE(x, -boxWidth);
      EXPECT_LE(x, boxWidth);
      EXPECT_GE(y, -boxWidth);
      EXPECT_LE(y, boxWidth);
    }
  }
  c.reset();

  delete [] depthImg;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, LensFlare)
{
  Load("worlds/lensflare_plugin.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Get the lens flare camera model
  std::string modelNameLensFlare = "camera_lensflare";
  std::string cameraNameLensFlare = "camera_sensor_lensflare";

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != nullptr);
  physics::ModelPtr model = world->ModelByName(modelNameLensFlare);
  ASSERT_TRUE(model != nullptr);

  sensors::SensorPtr sensorLensFlare =
    sensors::get_sensor(cameraNameLensFlare);
  sensors::CameraSensorPtr camSensorLensFlare =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorLensFlare);
  ASSERT_TRUE(camSensorLensFlare != nullptr);

  // Spawn a camera without lens flare at the same pose as
  // the camera lens flare model
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = camSensorLensFlare->ImageWidth();
  unsigned int height = camSensorLensFlare->ImageHeight();
  double updateRate = camSensorLensFlare->UpdateRate();

  EXPECT_GT(width, 0u);
  EXPECT_GT(height, 0u);
  EXPECT_GT(updateRate, 0u);

  ignition::math::Pose3d setPose = model->WorldPose();
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);

  // get a pointer to the camera lens flare sensor
  sensors::SensorPtr sensor =
    sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  ASSERT_TRUE(camSensor != nullptr);

  // collect images from both cameras
  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[width * height * 3];
  img2 = new unsigned char[width * height * 3];
  event::ConnectionPtr c =
    camSensorLensFlare->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c2 =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // Get some images
  int sleep = 0;
  while ((imageCount < 10 || imageCount2 < 10) && sleep++ < 1000)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, 10);
  EXPECT_GE(imageCount2, 10);

  // Compare colors. Camera sensor with lens flare plugin should have a brigher
  // image than the one without the lens flare plugin.
  unsigned int colorSum = 0;
  unsigned int colorSum2 = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      colorSum += r + g + b;
      unsigned int r2 = img2[(y*width*3) + x];
      unsigned int g2 = img2[(y*width*3) + x + 1];
      unsigned int b2 = img2[(y*width*3) + x + 2];
      colorSum2 += r2 + g2 + b2;
    }
  }
  EXPECT_GT(colorSum, colorSum2) <<
      "colorSum: " << colorSum << ", " <<
      "colorSum2: " << colorSum2;

  delete[] img;
  delete[] img2;
}
