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

// list of timestamped images used by the Timestamp test
std::vector<gazebo::msgs::ImageStamped> g_imagesStamped;

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
void OnImage(ConstImageStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  gazebo::msgs::ImageStamped imgStamped;
  imgStamped.CopyFrom(*_msg.get());
  g_imagesStamped.push_back(imgStamped);
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
  uint32_t width  = 320;
  uint32_t height = 240;
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

/////////////////////////////////////////////////
// Test the the SetCrop method for cameras works
// to set the distortion crop after creation of the sensor.
TEST_F(CameraSensor, CheckSetCrop)
{
  Load("worlds/empty.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Constants
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  const double k1 = -0.1349;
  const double k2 = -0.51868;
  const double k3 = -0.001;
  const int numImages = 10;
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));

  std::string names[] = {"barrel_default", "barrel_no_crop"};

  // Spawn two cameras with barrel distortion.
  // The first will crop border and the second will not
  int imageCounts[2];
  unsigned char* images[2];
  sensors::CameraSensorPtr cameras[2];
  event::ConnectionPtr connections[2];
  for (size_t i = 0; i < 2; ++i)
  {
      std::string modelName = names[i] + "_model";
      std::string sensorName = names[i] + "_sensor";
      SpawnCamera(modelName, sensorName, pose.Pos(),
                  pose.Rot().Euler(), width, height, updateRate,
                  "", 0, 0, true, k1, k2, k3, 0, 0, 0.5, 0.5);
      sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
      ASSERT_NE(sensor, nullptr);
      cameras[i] = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
      ASSERT_NE(cameras[i], nullptr);
      images[i] = new unsigned char[width * height * 3];
      imageCounts[i] = 0;
      connections[i] = cameras[i]->Camera()->ConnectNewImageFrame(
           std::bind(&::OnNewCameraFrame, &imageCounts[i], images[i],
                     std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3,
                     std::placeholders::_4, std::placeholders::_5));
      ASSERT_NE(connections[i], nullptr);
  }

  // Both cameras should start cropped by default
  EXPECT_TRUE(cameras[0]->Camera()->LensDistortion()->Crop());
  EXPECT_TRUE(cameras[1]->Camera()->LensDistortion()->Crop());

  // Set second camera to not crop
  cameras[1]->Camera()->LensDistortion()->SetCrop(false);
  EXPECT_FALSE(cameras[1]->Camera()->LensDistortion()->Crop());


  // Get some images
  // countdown timer to ensure test doesn't wait forever
  common::Timer timer(common::Time(10.0), true);
  timer.Start();
  while (imageCounts[0] < numImages || imageCounts[1] < numImages)
  {
    // Assert timeout has not passed
    ASSERT_NE(timer.GetElapsed(), common::Time::Zero);
    common::Time::MSleep(10);
  }

  unsigned int diffMax = 0, diffSum = 0;
  double diffAvg = 0.0;

  // We expect that there will be some non-zero difference between the images
  // because one should have a black border
  this->ImageCompare(images[0], images[1], width, height, 3,
                     diffMax, diffSum, diffAvg);
  EXPECT_NE(diffSum, 0u);

  // Sum the pixel values for each image
  uint64_t sums[2] = {0, 0};
  for (size_t i = 0; i < 2; ++i)
  {
    for (size_t y = 0; y < height; ++y)
    {
      for (size_t x = 0; x < width*3; x+=3)
      {
        size_t r = images[i][(y*width*3) + x];
        size_t g = images[i][(y*width*3) + x + 1];
        size_t b = images[i][(y*width*3) + x + 2];
        sums[i] += r + g + b;
      }
    }
  }

  // The image with the border cropped should be brighter
  // as it does not contain a black border
  EXPECT_GT(sums[0], sums[1]);

  // Delete heap allocated images
  for (size_t i = 0; i < 2; ++i)
    delete[] images[i];
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

  // get point cloud depth camera sensor
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

  // Get the lens flare camera model with scale applied
  std::string modelNameLensFlareScaled = "camera_lensflare_scaled";
  std::string cameraNameLensFlareScaled = "camera_sensor_lensflare_scaled";

  model = world->ModelByName(modelNameLensFlareScaled);
  ASSERT_TRUE(model != nullptr);

  sensors::SensorPtr sensorLensFlareScaled =
    sensors::get_sensor(cameraNameLensFlareScaled);
  sensors::CameraSensorPtr camSensorLensFlareScaled =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensorLensFlareScaled);
  ASSERT_TRUE(camSensorLensFlareScaled != nullptr);

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

  // get a pointer to the camera without lens flare
  sensors::SensorPtr sensor =
    sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  ASSERT_TRUE(camSensor != nullptr);

  // collect images from all 3 cameras
  imageCount = 0;
  imageCount2 = 0;
  imageCount3 = 0;
  img = new unsigned char[width * height * 3];
  img2 = new unsigned char[width * height * 3];
  img3 = new unsigned char[width * height * 3];
  event::ConnectionPtr c =
    camSensorLensFlare->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c2 =
    camSensorLensFlareScaled->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  event::ConnectionPtr c3 =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount3, img3,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // Get some images
  int sleep = 0;
  while ((imageCount < 10 || imageCount2 < 10 || imageCount3 < 10)
      && sleep++ < 1000)
  {
    common::Time::MSleep(10);
  }

  EXPECT_GE(imageCount, 10);
  EXPECT_GE(imageCount2, 10);
  EXPECT_GE(imageCount3, 10);

  c.reset();
  c2.reset();
  c3.reset();

  // Compare colors.
  unsigned int colorSum = 0;
  unsigned int colorSum2 = 0;
  unsigned int colorSum3 = 0;
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
      unsigned int r3 = img3[(y*width*3) + x];
      unsigned int g3 = img3[(y*width*3) + x + 1];
      unsigned int b3 = img3[(y*width*3) + x + 2];
      colorSum3 += r3 + g3 + b3;
    }
  }

  // camera with lens flare should be brighter than camera with scaled down
  // lens flare
  EXPECT_GT(colorSum, colorSum2) <<
      "colorSum: " << colorSum << ", " <<
      "colorSum2: " << colorSum2;
  // camera with scaled down lens flare should be brighter than camera without
  // lens flare
  EXPECT_GT(colorSum2, colorSum3) <<
      "colorSum2: " << colorSum2 << ", " <<
      "colorSum3: " << colorSum3;

  // test lens flare occlusion by spawning box in front of camera
  // Spawn a box in front of the cameras
  ignition::math::Vector3d boxPos = setPose.Pos()
      + ignition::math::Vector3d(2.5, 0, 0.5);
  SpawnBox("occlusion_box", ignition::math::Vector3d(0.5, 0.5, 0.5),
      boxPos, ignition::math::Vector3d::Zero, true);

  c = camSensorLensFlare->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  c2 = camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // Get more images
  sleep = 0;
  imageCount = 0;
  imageCount2 = 0;
  while ((imageCount < 10 || imageCount2 < 10) && sleep++ < 1000)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, 10);
  EXPECT_GE(imageCount2, 10);

  // Lens flare should be completely occluded.
  // Camera sensor with lens flare plugin should have approx the same image as
  // the one without the lens flare plugin.
  colorSum = 0;
  colorSum2 = 0;
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

  // set tolerance to be 0.02% of total pixel values
  unsigned int tol = width * height * 3 * 255 * 2e-4;
  EXPECT_NEAR(colorSum, colorSum2, tol);

  delete[] img;
  delete[] img2;
  delete[] img3;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, 16bit)
{
  // World contains a box positioned at top right quadrant of image generated
  // by a mono 16 bit camera and a color 16 bit camera.
  // Verify pixel values of top right quadrant of image (corresponding to box)
  // are approximately the same but different from the background's pixel
  // values.
  Load("worlds/16bit_camera.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // get L16 camera sensor
  std::string l16CameraName = "l16bit_camera_sensor";
  sensors::SensorPtr l16Sensor = sensors::get_sensor(l16CameraName);
  sensors::CameraSensorPtr l16CamSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(l16Sensor);
  EXPECT_TRUE(l16CamSensor != nullptr);
  rendering::CameraPtr l16Cam = l16CamSensor->Camera();
  EXPECT_TRUE(l16Cam != nullptr);

  unsigned int l16Width  = l16Cam->ImageWidth();
  unsigned int l16Height = l16Cam->ImageHeight();
  EXPECT_GT(l16Width, 0u);
  EXPECT_GT(l16Height, 0u);

  // get rgb16 camera sensor
  std::string rgb16CameraName = "rgb16bit_camera_sensor";
  sensors::SensorPtr rgb16Sensor = sensors::get_sensor(rgb16CameraName);
  sensors::CameraSensorPtr rgb16CamSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(rgb16Sensor);
  EXPECT_TRUE(rgb16CamSensor != nullptr);
  rendering::CameraPtr rgb16Cam = rgb16CamSensor->Camera();
  EXPECT_TRUE(rgb16Cam != nullptr);

  unsigned int rgb16Width  = rgb16Cam->ImageWidth();
  unsigned int rgb16Height = rgb16Cam->ImageHeight();
  EXPECT_GT(rgb16Width, 0u);
  EXPECT_GT(rgb16Height, 0u);

  // connect to new frame event
  imageCount = 0;
  imageCount2 = 0;
  img = new unsigned char[l16Width * l16Height * 2];
  img2 = new unsigned char[rgb16Width * rgb16Height * 3 * 2];

  event::ConnectionPtr c =
    l16CamSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  event::ConnectionPtr c2 =
    rgb16CamSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // wait for a few images
  int sleep = 0;
  int maxSleep = 500;
  int totalImages = 10;
  while ((imageCount < totalImages || imageCount2 < totalImages)
      && sleep++ < maxSleep)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, totalImages);
  EXPECT_GE(imageCount2, totalImages);

  c.reset();
  c2.reset();

  // verify L16 camera images
  uint16_t bgValue = 0;
  uint16_t boxValue = 0;
  uint16_t *l16Img = reinterpret_cast<uint16_t *>(img);
  // expect pixel values to be within 0.2% of valid 16bit pixel range
  uint16_t tol = std::pow(2, 16) * 0.002;
  for (unsigned int y = 0; y < l16Height; ++y)
  {
    for (unsigned int x = 0; x < l16Width; ++x)
    {
      uint16_t value = l16Img[(y*l16Width)+x];
      EXPECT_NE(0, value);

      // box in top right quadrant of image
      if (x >= l16Width / 2 && y < l16Height / 2)
      {
        // set its color if not done already
        if (boxValue == 0)
          boxValue = value;
        // verify pixels correspond to box
        else
          EXPECT_NEAR(boxValue, value, tol);
      }
      // rest are all background
      else
      {
        // set background color if not done already
        if (bgValue == 0)
          bgValue = value;
        // verify pixels correspond to background
        else
          EXPECT_NEAR(bgValue, value, tol);
      }
    }
  }

  // expect background pixel value to be different from box pixel value
  EXPECT_GT(bgValue, boxValue);
  uint16_t minDiff = std::pow(2, 16) * 0.25;
  uint16_t diff = bgValue - boxValue;
  EXPECT_GT(diff, minDiff);


  // verify RGB UINT16 camera images
  uint16_t bgRValue = 0;
  uint16_t bgGValue = 0;
  uint16_t bgBValue = 0;
  uint16_t boxRValue = 0;
  uint16_t boxGValue = 0;
  uint16_t boxBValue = 0;
  uint16_t *rgb16Img = reinterpret_cast<uint16_t *>(img2);
  for (unsigned int y = 0; y < rgb16Height; ++y)
  {
    for (unsigned int x = 0; x < rgb16Width * 3; x+=3)
    {
      uint16_t r = rgb16Img[(y*rgb16Width*3)+x];
      uint16_t g = rgb16Img[(y*rgb16Width*3)+x+1];
      uint16_t b = rgb16Img[(y*rgb16Width*3)+x+2];
      // verify gray color
      EXPECT_EQ(r, g);
      EXPECT_EQ(r, b);
      EXPECT_NE(0, r);
      EXPECT_NE(0, g);
      EXPECT_NE(0, b);

      // box in top right quadrant of image
      if (x >= (rgb16Width*3) / 2 && y < rgb16Height / 2)
      {
        // set its color if not done already
        if (boxRValue == 0 && boxGValue == 0 && boxBValue == 0)
        {
          boxRValue = r;
          boxGValue = g;
          boxBValue = b;
        }
        // verify pixels correspond to box
        else
        {
          EXPECT_NEAR(boxRValue, r, tol);
          EXPECT_NEAR(boxGValue, g, tol);
          EXPECT_NEAR(boxBValue, b, tol);
        }
      }
      // rest are all background
      else
      {
        // set background color if not done already
        if (bgRValue == 0 && bgGValue == 0 && bgBValue == 0)
        {
          bgRValue = r;
          bgGValue = g;
          bgBValue = b;
        }
        // verify pixels correspond to background
        else
        {
          EXPECT_NEAR(bgRValue, r, tol);
          EXPECT_NEAR(bgGValue, g, tol);
          EXPECT_NEAR(bgBValue, b, tol);
        }
      }
    }
  }
  // expect background color to be different from box color
  EXPECT_GT(bgRValue, boxRValue);
  EXPECT_GT(bgGValue, boxGValue);
  EXPECT_GT(bgBValue, boxBValue);
  uint16_t diffR = bgRValue - boxRValue;
  uint16_t diffG = bgGValue - boxGValue;
  uint16_t diffB = bgBValue - boxBValue;
  EXPECT_GT(diffR, minDiff);
  EXPECT_GT(diffG, minDiff);
  EXPECT_GT(diffB, minDiff);

  delete [] img;
  delete [] img2;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, AmbientOcclusion)
{
  Load("worlds/ssao_plugin.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // spawn a camera
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose(
      ignition::math::Vector3d(6, 0, 2),
      ignition::math::Quaterniond(0, 0, 3.14));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  // collect images
  imageCount = 0;
  img = new unsigned char[width * height * 3];
  event::ConnectionPtr c =
    camSensor->Camera()->ConnectNewImageFrame(
        std::bind(&::OnNewCameraFrame, &imageCount, img,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
  // Get some images
  int sleep = 0;
  while ((imageCount < 10) && sleep++ < 1000)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, 10);

  // verify image contains gray pixels
  // Without the ambient occlusion plugin, it would just be a white image.
  std::map<unsigned int, unsigned int> uniquePixel;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      EXPECT_EQ(r, g);
      EXPECT_EQ(r, b);
      if (uniquePixel.find(r) != uniquePixel.end())
        uniquePixel[r] = ++uniquePixel[r];
      else
        uniquePixel[r] = 1;
    }
  }
  // verify image is predominantly white but not the whole image
  EXPECT_GT(uniquePixel[255], width*height*0.80);
  EXPECT_LT(uniquePixel[255], width*height*0.85);
  // there should be some variations of grayscale pixels
  EXPECT_LT(uniquePixel.size(), 255*0.35);
  delete[] img;
}

/////////////////////////////////////////////////
// Move a tall thin box across the center of the camera image
// (from -y to +y) over time and collect camera sensor timestamped images.
// For every image collected, extract center of box from image, and compare it
// against analytically computed box position.
TEST_F(CameraSensor, Timestamp)
{
  this->Load("worlds/empty_test.world", true);

  // Make sure the render engine is available.
  ASSERT_TRUE(rendering::RenderEngine::Instance()->GetRenderPathType() !=
      rendering::RenderEngine::NONE);

  // variables for testing
  // camera image width
  unsigned int width  = 240;
  // camera image height
  unsigned int height = 160;
  unsigned int halfHeight = height * 0.5;
  // camera sensor update rate
  double sensorUpdateRate = 10;
  // Speed at which the box is moved, in meters per second
  double boxMoveVel = 1.0;

  // world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // set gravity to 0, 0, 0
  world->SetGravity(ignition::math::Vector3d::Zero);
  EXPECT_EQ(world->Gravity(), ignition::math::Vector3d::Zero);

  // spawn camera sensor
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  ignition::math::Pose3d setPose(
      ignition::math::Vector3d(-5, 0, 0),
      ignition::math::Quaterniond::Identity);
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, sensorUpdateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(camSensor != nullptr);
  camSensor->SetActive(true);
  EXPECT_TRUE(camSensor->IsActive());

  // spawn a tall thin box in front of camera but out of its view at neg y;
  std::string boxName = "box_0";
  double initDist = -3;
  ignition::math::Pose3d boxPose(0, initDist, 0.0, 0, 0, 0);
  SpawnBox(boxName, ignition::math::Vector3d(0.005, 0.005, 0.1), boxPose.Pos(),
      boxPose.Rot().Euler());

  gazebo::physics::ModelPtr boxModel = world->ModelByName(boxName);
  EXPECT_TRUE(boxModel != nullptr);

  // step 100 times - this will be the start time for our experiment
  int startTimeIt = 100;
  world->Step(startTimeIt);

  // clear the list of timestamp images
  g_imagesStamped.clear();

  // verify that time moves forward
  double t = world->SimTime().Double();
  EXPECT_GT(t, 0);

  // subscribe to camera topic and collect timestamp images
  std::string cameraTopic = camSensor->Topic();
  EXPECT_TRUE(!cameraTopic.empty());
  transport::SubscriberPtr sub = this->node->Subscribe(cameraTopic, OnImage);

  // get physics engine
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);

  // move the box for a period of 6 seconds along +y
  unsigned int period = 6;
  double stepSize = physics->GetMaxStepSize();
  unsigned int iterations = static_cast<unsigned int>(period*(1.0/stepSize));

  for (unsigned int i = 0; i < iterations; ++i)
  {
    double dist = (i+1)*(boxMoveVel*stepSize);
    // move the box along y
    boxModel->SetWorldPose(
        ignition::math::Pose3d(0, initDist+ dist, 0, 0, 0, 0));
    world->Step(1);
    EXPECT_EQ(boxModel->WorldPose().Pos().Y(), initDist + dist);
  }

  // wait until we get all timestamp images
  int sleep = 0;
  int maxSleep = 50;
  unsigned int imgSampleSize = period / (1.0 / sensorUpdateRate);
  while (sleep < maxSleep)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (g_imagesStamped.size() >= imgSampleSize)
      break;
    sleep++;
    gazebo::common::Time::MSleep(10);
  }

  // stop the camera subscriber
  sub.reset();

  // compute expected 2D pos of box and compare it against the
  // actual pos of the box found in the timestamp images.
  unsigned int imgSize = width * height * 3;
  img = new unsigned char[imgSize];
  for (const auto &msg : g_imagesStamped)
  {
    // time t
    gazebo::common::Time timestamp = gazebo::msgs::Convert(msg.time());
    double t = timestamp.Double();

    // calculate expected box pose at time=t
    int it = t * (1.0 / stepSize) - startTimeIt;
    double dist = it*(boxMoveVel*stepSize);
    // project box 3D pos to screen space
    ignition::math::Vector2i p2 = camSensor->Camera()->Project(
        ignition::math::Vector3d(0, initDist + dist, 0));

    // find actual box pose at time=t
    // walk along the middle row of the img and identify center of box
    int left = -1;
    int right = -1;
    bool transition = false;
    memcpy(img, msg.image().data().c_str(), imgSize);
    for (unsigned int i = 0; i < width; ++i)
    {
      int row = halfHeight * width * 3;
      int r = img[row + i*3];
      int g = img[row + i*3+1];
      int b = img[row + i*3+2];

      // bg color determined experimentally
      int bgColor = 178;

      if (r < bgColor && g < bgColor && b < bgColor)
      {
        if (!transition)
        {
          left = i;
          transition = true;
        }
      }
      else if (transition)
      {
        right = i-1;
        break;
      }
    }

    // if box is out of camera view, expect no box found in image
    if (p2.X() < 0 || p2.X () > static_cast<int>(width))
    {
      EXPECT_TRUE(left < 0 || right < 0)
          << "Expected box pos: " << p2 << "\n"
          << "Actual box left: " << left << ", right: " << right;
    }
    else
    {
      double mid = -1;
      // left and right of box found in image
      if (left >= 0 && right >= 0)
      {
        mid = (left + right) * 0.5;
      }
      // edge case - box at edge of image
      else if (left >= 0 && right < 0)
      {
        mid = left;
      }
      else
      {
        FAIL() << "No box found in image.\n"
               << "time: " << t << "\n"
               << "Expected box pos: " << p2 << "\n"
               << "Actual box left: " << left << ", right: " << right;
      }

      EXPECT_GE(mid, 0);

      // expected box pos should roughly be equal to actual box pos +- 1 pixel
      EXPECT_NEAR(mid, p2.X(), 1.0) << "Expected box pos: " << p2 << "\n"
          << "Actual box left: " << left << ", right: " << right;
    }
  }

  delete [] img;
}

/////////////////////////////////////////////////
TEST_F(CameraSensor, Light)
{
  Load("worlds/empty.world");

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
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d setPose, testPose(
      ignition::math::Vector3d(-5, 0, 5),
      ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.Pos(),
      setPose.Rot().Euler(), width, height, updateRate);
  std::string sensorScopedName =
      "default::" + modelName + "::body::" + cameraName;
  sensors::SensorPtr sensor = sensors::get_sensor(sensorScopedName);
  EXPECT_TRUE(sensor != nullptr);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  ASSERT_TRUE(camSensor != nullptr);
  rendering::CameraPtr camera = camSensor->Camera();
  ASSERT_TRUE(camera != nullptr);

  // get camera scene
  rendering::ScenePtr scene = camera->GetScene();
  ASSERT_NE(nullptr, scene);

  transport::PublisherPtr lightModifyPub = this->node->Advertise<msgs::Light>(
        "~/light/modify");

  // Set the light to be green
  ignition::math::Color newColor(0, 1, 0);
  msgs::Light lightMsg;
  lightMsg.set_name("sun");
  msgs::Set(lightMsg.mutable_diffuse(), newColor);
  lightModifyPub->Publish(lightMsg);

  rendering::LightPtr sun = scene->LightByName("sun");
  ASSERT_TRUE(sun != nullptr);

  int sleep = 0;
  while (sun->DiffuseColor() != newColor && sleep++ < 50)
  {
    common::Time::MSleep(100);
  }
  EXPECT_EQ(newColor, sun->DiffuseColor());
}
