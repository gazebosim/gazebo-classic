/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class VisualProperty : public ServerFixture
{
};

std::mutex mutex;

unsigned char* img = nullptr;
unsigned char* img2 = nullptr;
int imageCount = 0;
int imageCount2 = 0;

/////////////////////////////////////////////////
void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(mutex);
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(VisualProperty, CastShadows)
{
  Load("worlds/visual_shadows.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test"
          << std::endl;
    return;
  }

  physics::WorldPtr world = physics::get_world();

  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;

  // spawn first camera sensor
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.5),
      ignition::math::Quaterniond(0, 1.57, 0));
  SpawnCamera(modelName, cameraName, testPose.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  physics::ModelPtr model = world->GetModel(modelName);
  EXPECT_EQ(model->GetWorldPose().Ign(), testPose);

  imageCount = 0;
  img = new unsigned char[width * height * 3];

  event::ConnectionPtr c =
      camSensor->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  common::Timer timer;
  timer.Start();

  // wait for images
  int totalImages = 20;
  while (imageCount < totalImages && timer.GetElapsed().Double() < 5)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, totalImages);
  camSensor->Camera()->DisconnectNewImageFrame(c);

  // spawn second camera sensor
  ignition::math::Pose3d testPose2(
      ignition::math::Vector3d(0, 10, 0.5),
      ignition::math::Quaterniond(0, 1.57, 0));
  std::string modelName2 = "camera_model2";
  std::string cameraName2 = "camera_sensor2";
  SpawnCamera(modelName2, cameraName2, testPose2.Pos(),
      testPose2.Rot().Euler(), width, height, updateRate);

  sensors::SensorPtr sensor2 = sensors::get_sensor(cameraName2);
  sensors::CameraSensorPtr camSensor2 =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);

  physics::ModelPtr model2 = world->GetModel(modelName2);
  EXPECT_EQ(model2->GetWorldPose().Ign(), testPose2);

  imageCount2 = 0;
  img2 = new unsigned char[width * height * 3];

  event::ConnectionPtr c2 =
      camSensor2->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount2, img2,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  common::Timer timer2;
  timer2.Start();

  while (imageCount2 < totalImages && timer2.GetElapsed().Double() < 5)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount2, totalImages);
  camSensor2->Camera()->DisconnectNewImageFrame(c2);

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

  // camera1 image should be darker than camera2 image
  // because the mesh below camera1 is casting shadows
  EXPECT_LT(colorSum, colorSum2);
  double colorRatio = static_cast<double>(colorSum2-colorSum) /
      static_cast<double>(colorSum2);
  EXPECT_GT(colorRatio, 0.05)
    << " colorSum [" << colorSum << "], "
    << " colorSum2 [" << colorSum2 << "]";

  delete [] img;
  delete [] img2;
}

bool g_shaderParamSet = false;

/////////////////////////////////////////////////
void SetShaderParam(const std::string &_visualName,
    const std::string &_paramName, const std::string &_shaderType,
    const std::string &_value)
{
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(nullptr, scene);

  rendering::VisualPtr visual = scene->GetVisual(_visualName);
  ASSERT_NE(nullptr, visual);

  // change shader param value
  visual->SetMaterialShaderParam(_paramName, _shaderType, _value);

  g_shaderParamSet = true;
}


/////////////////////////////////////////////////
TEST_F(VisualProperty, MaterialShaderParam)
{
  // Load a world with a camera facing a red box
  // This test verifies the box visual color can be changed to green by
  // setting the `color` uniform parameter exposed by the fragment shader
  // The box visual's material and shader files are shader_test.material,
  // shader_test_vp.glsl, shader_test_fp.glsl in test/media/materials/scripts
  // directory.
  Load("worlds/shader_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test"
          << std::endl;
    return;
  }
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(nullptr, scene);

  // 1 camera in scene
  rendering::CameraPtr cam = scene->GetCamera(0);
  ASSERT_NE(nullptr, cam);

  int totalImages = 20;
  imageCount = 0;
  unsigned int width = cam->ImageWidth();
  unsigned int height = cam->ImageHeight();
  img = new unsigned char[width * height * 3];

  event::ConnectionPtr c =
      cam->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  unsigned int sleep = 0;
  unsigned int maxSleep = 50;
  while (imageCount < totalImages && sleep++ < maxSleep)
    common::Time::MSleep(100);

  EXPECT_GE(imageCount, totalImages);
  cam->DisconnectNewImageFrame(c);

  // check initial color
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      EXPECT_EQ(255u, r);
      EXPECT_EQ(0u, g);
      EXPECT_EQ(0u, b);
    }
  }

  // now set shader material param in rendering thread
  g_shaderParamSet = false;
  std::string visualName = "box::link::visual";
  std::string paramName = "color";
  std::string shaderType = "fragment";
  std::string value = "0 1 0 1";
  // Connect to the render signal
  auto c2 =
      event::Events::ConnectPreRender(std::bind(&::SetShaderParam,
      visualName, paramName, shaderType, value));

  // wait for the param to be set
  sleep = 0u;
  while (!g_shaderParamSet && sleep++ < maxSleep)
    common::Time::MSleep(100);

  EXPECT_TRUE(g_shaderParamSet);

  c2.reset();

  // get more images
  sleep = 0;
  imageCount = 0;
  c = cam->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  while (imageCount < totalImages && sleep++ < maxSleep)
    common::Time::MSleep(100);

  EXPECT_GE(imageCount, totalImages);
  cam->DisconnectNewImageFrame(c);

  // verify new color has been set
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      EXPECT_EQ(0u, r);
      EXPECT_EQ(255u, g);
      EXPECT_EQ(0u, b);
    }
  }

  delete [] img;
}

/////////////////////////////////////////////////
// normal is not work on OSX yet
#ifndef __APPLE__
TEST_F(VisualProperty, NormalMap)
{
  // Load a world with two red box visuals: one without normal and and one with
  // normal map. Spawn a camera in front of each visual. Verify the color of
  // visual with normal mpa is different (darker) than visual without normal
  // map and has  irregular pattern
  Load("worlds/normal_map.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test"
          << std::endl;
    return;
  }

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != nullptr);

  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;

  // spawn first camera sensor
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, -1, 0.5),
      ignition::math::Quaterniond(0, 0, 1.57));
  SpawnCamera(modelName, cameraName, testPose.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  physics::ModelPtr model = world->GetModel(modelName);
  EXPECT_EQ(model->GetWorldPose().Ign(), testPose);

  imageCount = 0;
  img = new unsigned char[width * height * 3];

  event::ConnectionPtr c =
      camSensor->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  common::Timer timer;
  timer.Start();

  // wait for images
  int totalImages = 20;
  while (imageCount < totalImages && timer.GetElapsed().Double() < 5)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount, totalImages);
  camSensor->Camera()->DisconnectNewImageFrame(c);

  // spawn second camera sensor
  ignition::math::Pose3d testPose2(
      ignition::math::Vector3d(3, -1, 0.5),
      ignition::math::Quaterniond(0, 0, 1.57));
  std::string modelName2 = "camera_model2";
  std::string cameraName2 = "camera_sensor2";
  SpawnCamera(modelName2, cameraName2, testPose2.Pos(),
      testPose2.Rot().Euler(), width, height, updateRate);

  sensors::SensorPtr sensor2 = sensors::get_sensor(cameraName2);
  sensors::CameraSensorPtr camSensor2 =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);

  physics::ModelPtr model2 = world->GetModel(modelName2);
  EXPECT_EQ(model2->GetWorldPose().Ign(), testPose2);

  imageCount2 = 0;
  img2 = new unsigned char[width * height * 3];

  event::ConnectionPtr c2 =
      camSensor2->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount2, img2,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  common::Timer timer2;
  timer2.Start();

  while (imageCount2 < totalImages && timer2.GetElapsed().Double() < 5)
    common::Time::MSleep(10);

  EXPECT_GE(imageCount2, totalImages);
  camSensor2->Camera()->DisconnectNewImageFrame(c2);

  // check color of visuals with and without normal map
  std::set<unsigned int> rSet;
  std::set<unsigned int> gSet;
  std::set<unsigned int> bSet;
  std::set<unsigned int> rSet2;
  std::set<unsigned int> gSet2;
  std::set<unsigned int> bSet2;
  unsigned int colorSum = 0;
  unsigned int colorSum2 = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*3; x+=3)
    {
      // visual without normal map
      unsigned int r = img[(y*width*3) + x];
      unsigned int g = img[(y*width*3) + x + 1];
      unsigned int b = img[(y*width*3) + x + 2];
      rSet.insert(r);
      gSet.insert(g);
      bSet.insert(b);
      colorSum += r + g + b;
      // verify color is predominantly red
      EXPECT_GT(r, g);
      EXPECT_GT(r, b);
      EXPECT_EQ(g, b);

      // visual with normal map
      unsigned int r2 = img2[(y*width*3) + x];
      unsigned int g2 = img2[(y*width*3) + x + 1];
      unsigned int b2 = img2[(y*width*3) + x + 2];
      rSet2.insert(r2);
      gSet2.insert(g2);
      bSet2.insert(b2);
      colorSum2 += r2+ g2+ b2;

      // verify color is predominantly red
      EXPECT_GT(r2, g2);
      EXPECT_GT(r2, b2);
      EXPECT_EQ(g2, b2);
    }
  }
  // verify the rgb components of pixel are somewhat consistent throughout the
  // the image
  EXPECT_LE(rSet.size(), 3u);
  EXPECT_LE(gSet.size(), 3u);
  EXPECT_LE(bSet.size(), 3u);

  // check that the r component of pixel varies throughout the image for
  // visual with normal map. The variation of b and g should still be small
  EXPECT_GT(rSet2.size(), 150u);
  EXPECT_LE(gSet2.size(), 5u);
  EXPECT_LE(bSet2.size(), 5u);

  // the visual with normal map should be darker than the visual without
  // normal map
  EXPECT_GT(colorSum, colorSum2);

  delete [] img;
  delete [] img2;
}
#endif

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
