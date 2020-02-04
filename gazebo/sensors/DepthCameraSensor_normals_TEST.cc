/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <functional>
#include <mutex>

#include <gtest/gtest.h>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class DepthCameraSensor_normals_TEST : public ServerFixture
{
};
float farClip;

/////////////////////////////////////////////////
void OnNewNormalsFrame(const float * _normals,
                       unsigned int _width,
                       unsigned int _height,
                       unsigned int _depth,
                       const std::string &/*_format*/)
{
  int index =  ((_height * 0.5) * _width) + _width * 0.5;

  for (unsigned int i = 0; i < _width; i++)
  {
    for (unsigned int j = 0; j < _height; j++)
    {
      unsigned int index = (j * _width) + i;
      float x = _normals[4 * index];
      float y = _normals[4 * index + 1];
      float z = _normals[4 * index + 2];
      float a = _normals[4 * index + 3];
      // if(x < farClip)
      //   printf("W[%u] H[%u] %f\t%f\t%f\t%f\n", _width, _height, x, y, z, a);
    }
  }
}

/////////////////////////////////////////////////
/// \brief Test Creation of a Depth Camera sensor
TEST_F(DepthCameraSensor_normals_TEST, CreateDepthCamera)
{
  Load("worlds/depth_camera2.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the camera sensor
  std::string sensorName = "default::camera_model::my_link::camera";

  // Get a pointer to the depth camera sensor
  sensors::DepthCameraSensorPtr sensor =
     std::dynamic_pointer_cast<sensors::DepthCameraSensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(sensor->ImageWidth(), 640u);
  EXPECT_EQ(sensor->ImageHeight(), 480u);
  EXPECT_TRUE(sensor->IsActive());

  rendering::DepthCameraPtr depthCamera = sensor->DepthCamera();
  EXPECT_TRUE(depthCamera != nullptr);

  farClip = depthCamera->FarClip();

  event::ConnectionPtr c2 = depthCamera->ConnectNewNormalsPointCloud(
      std::bind(&::OnNewNormalsFrame, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
      std::placeholders::_5));

  // wait for a few depth camera frames
  int i = 0;
  while ( i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
