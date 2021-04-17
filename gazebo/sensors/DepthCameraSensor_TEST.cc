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

#include <functional>
#include <mutex>

#include <gtest/gtest.h>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class DepthCameraSensor_TEST : public ServerFixture
{
  public: DepthCameraSensor_TEST();
  public: void TestCreateDepthCameraSensor();
  public: void TestDepthCameraReflectance();
  public: void TestDepthCameraNormals();
  public: void TestReflectanceTopic();
  private: void OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);
  private: void OnNewReflectanceFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);
  private: void OnNewNormalsFrame(const float *_normals,
                       unsigned int _width,
                       unsigned int _height,
                       unsigned int _depth,
                       const std::string &_format);
  private: void TxMsg(const ConstPropagationGridPtr &_msg);

  private: std::mutex depthMutex;
  private: unsigned int depthCounter;
  private: float *depthBuffer;
  private: std::mutex reflectanceMutex;
  private: unsigned int reflectanceCounter;
  private: float *reflectanceBuffer;
  private: unsigned int normalsCounter;
  private: std::mutex reflectanceTopicMutex;
  private: bool reflectanceTopicReceivedMsg;
  private: boost::shared_ptr<msgs::PropagationGrid const>
      reflectanceTopicGridMsg;
};

DepthCameraSensor_TEST::DepthCameraSensor_TEST()
{
  this->depthCounter = 0;
  this->depthBuffer = nullptr;
  this->reflectanceCounter = 0;
  this->reflectanceBuffer = nullptr;
  this->normalsCounter = 0;
  this->reflectanceTopicReceivedMsg = false;
}

/////////////////////////////////////////////////
void DepthCameraSensor_TEST::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format)
{
  EXPECT_EQ(_depth, 1u);
  EXPECT_EQ(_format, std::string("FLOAT32"));
  ASSERT_NE(nullptr, _image);
  std::lock_guard<std::mutex> lock(this->depthMutex);
  if (!this->depthBuffer)
    this->depthBuffer = new float[_width * _height];
  memcpy(this->depthBuffer,  _image, _width * _height * sizeof(_image[0]));
  this->depthCounter++;
}

/////////////////////////////////////////////////
void DepthCameraSensor_TEST::TestCreateDepthCameraSensor()
{
  Load("worlds/depth_camera.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the camera sensor
  std::string sensorName = "default::camera_model::my_link::camera";

  // Get a pointer to the depth camera sensor
  sensors::DepthCameraSensorPtr sensor =
     std::dynamic_pointer_cast<sensors::DepthCameraSensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(sensor->ImageWidth(), 640u);
  EXPECT_EQ(sensor->ImageHeight(), 480u);
  EXPECT_TRUE(sensor->IsActive());

  rendering::DepthCameraPtr depthCamera = sensor->DepthCamera();
  ASSERT_NE(nullptr, depthCamera);

  event::ConnectionPtr c = depthCamera->ConnectNewDepthFrame(
      std::bind(&DepthCameraSensor_TEST::OnNewDepthFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  // wait for a few depth camera frames
  unsigned int framesToWait = 10;
  int i = 0;
  while (i < 300 && this->depthCounter < framesToWait)
  {
    common::Time::MSleep(20);
    i++;
  }
  EXPECT_GE(this->depthCounter, framesToWait);

  unsigned int imageSize =
      sensor->ImageWidth() * sensor->ImageHeight();

  std::lock_guard<std::mutex> lock(this->depthMutex);
  // Check that the depth values are valid
  for (unsigned int i = 0; i < imageSize; ++i)
  {
    EXPECT_TRUE(this->depthBuffer[i] <= depthCamera->FarClip());
    EXPECT_TRUE(this->depthBuffer[i] >= depthCamera->NearClip());
    EXPECT_TRUE(!ignition::math::equal(this->depthBuffer[i], 0.0f));
  }

  // sphere with radius 1m is at 2m in front of depth camera
  // so verify depth readings are between 1-2m in the mid row
  unsigned int halfHeight =
    static_cast<unsigned int>(sensor->ImageHeight()*0.5)-1;
  for (unsigned int i = sensor->ImageWidth()*halfHeight;
      i < sensor->ImageWidth()*(halfHeight+1); ++i)
  {
    EXPECT_TRUE(this->depthBuffer[i] < 2.0f);
    EXPECT_TRUE(this->depthBuffer[i] >= 1.0f);
  }

  if (this->depthBuffer)
    delete [] this->depthBuffer;
}

/////////////////////////////////////////////////
void DepthCameraSensor_TEST::OnNewReflectanceFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format)
{
  EXPECT_EQ(_depth, 1u);
  EXPECT_EQ(_format, std::string("REFLECTANCE"));
  ASSERT_NE(nullptr, _image);
  std::lock_guard<std::mutex> lock(this->reflectanceMutex);
  if (!this->reflectanceBuffer)
    this->reflectanceBuffer = new float[_width * _height];
  memcpy(this->reflectanceBuffer,
      _image, _width * _height * sizeof(_image[0]));
  this->reflectanceCounter++;
}

/////////////////////////////////////////////////
/// \brief Test Creation of a Depth Camera sensor
void DepthCameraSensor_TEST::TestDepthCameraReflectance()
{
  Load("worlds/reflectance.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Create the camera sensor
  std::string sensorName = "default::camera_model::my_link::camera";

  // Get a pointer to the depth camera sensor
  sensors::DepthCameraSensorPtr sensor =
     std::dynamic_pointer_cast<sensors::DepthCameraSensor>
     (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(sensor->ImageWidth(), 640u);
  EXPECT_EQ(sensor->ImageHeight(), 480u);
  EXPECT_TRUE(sensor->IsActive());

  rendering::DepthCameraPtr depthCamera = sensor->DepthCamera();
  ASSERT_NE(nullptr, depthCamera);

  event::ConnectionPtr c = depthCamera->ConnectNewReflectanceFrame(
      std::bind(&DepthCameraSensor_TEST::OnNewReflectanceFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  // wait for a few depth camera frames
  unsigned int framesToWait = 10;
  int i = 0;
  while (i < 300 && this->reflectanceCounter < framesToWait)
  {
    common::Time::MSleep(20);
    i++;
  }
  EXPECT_GE(this->reflectanceCounter, framesToWait);

  std::lock_guard<std::mutex> lock(this->reflectanceMutex);

  // Check the reflectance of the box
  unsigned int index = ((sensor->ImageHeight() * 0.45) * sensor->ImageWidth())
                       + sensor->ImageWidth() * 0.5;
  for (unsigned int i = index - 25;
      i < index + 25 ; ++i)
  {
    EXPECT_GT(this->reflectanceBuffer[i], 0);
  }

  depthCamera.reset();

  if (this->reflectanceBuffer)
    delete [] this->reflectanceBuffer;
}

/////////////////////////////////////////////////
void DepthCameraSensor_TEST::OnNewNormalsFrame(const float *_normals,
                       unsigned int _width,
                       unsigned int _height,
                       unsigned int _depth,
                       const std::string &_format)
{
  EXPECT_EQ(_depth, 1u);
  EXPECT_EQ(_format, std::string("NORMALS"));
  for (unsigned int i = 0; i < _width; i++)
  {
    for (unsigned int j = 0; j < _height; j++)
    {
      unsigned int index = (j * _width) + i;
      float x = _normals[4 * index];
      float y = _normals[4 * index + 1];
      float z = _normals[4 * index + 2];
      EXPECT_NEAR(x, 0.0, 0.01);
      EXPECT_NEAR(y, 0.0, 0.01);
      // box
      if (z < -0.5)
        EXPECT_NEAR(z, -1.0, 0.01);
      // background
      else
        EXPECT_NEAR(z, 0.0, 0.01);
    }
  }
  this->normalsCounter++;
}

/////////////////////////////////////////////////
/// \brief Test Creation of a Depth Camera sensor
void DepthCameraSensor_TEST::TestDepthCameraNormals()
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
  ASSERT_NE(nullptr, sensor);

  EXPECT_EQ(sensor->ImageWidth(), 640u);
  EXPECT_EQ(sensor->ImageHeight(), 480u);
  EXPECT_TRUE(sensor->IsActive());

  rendering::DepthCameraPtr depthCamera = sensor->DepthCamera();
  ASSERT_NE(nullptr, depthCamera);

  event::ConnectionPtr c2 = depthCamera->ConnectNewNormalsPointCloud(
      std::bind(&DepthCameraSensor_TEST::OnNewNormalsFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  unsigned int framesToWait = 10;
  // wait for a few normals callbacks
  int i = 0;
  while (i < 300 && this->normalsCounter < framesToWait)
  {
    common::Time::MSleep(20);
    i++;
  }
  EXPECT_GE(this->normalsCounter, framesToWait);

  depthCamera.reset();
}

/////////////////////////////////////////////////
/// \brief Callback executed for every propagation grid message received
void DepthCameraSensor_TEST::TxMsg(const ConstPropagationGridPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->reflectanceTopicMutex);
  // Just copy the message
  this->reflectanceTopicGridMsg = _msg;
  this->reflectanceTopicReceivedMsg = true;
}

/////////////////////////////////////////////////
/// \brief Test the publication of the reflectance image used for visualization
void DepthCameraSensor_TEST::TestReflectanceTopic()
{
  {
    Load("worlds/reflectance.world");

    // Initialize gazebo transport layer
    transport::NodePtr node(new transport::Node());
    node->Init("default");

    std::string txTopic =
        "/gazebo/default/camera_model/my_link/camera/image_reflectance";
    transport::SubscriberPtr sub = node->Subscribe(txTopic,
        &DepthCameraSensor_TEST::TxMsg, this);

    // Make sure that the sensor is updated and some messages are published
    for (int i = 0; i < 10; ++i)
    {
      common::Time::MSleep(100);
    }

    std::lock_guard<std::mutex> lock(this->reflectanceTopicMutex);
    EXPECT_TRUE(this->reflectanceTopicReceivedMsg);
  }

  this->reflectanceTopicReceivedMsg = false;
  Unload();

  {
    Load("worlds/depth_camera.world");

    // Initialize gazebo transport layer
    transport::NodePtr node(new transport::Node());
    node->Init("default");

    std::string txTopic =
        "/gazebo/default/camera_model/my_link/camera/image_reflectance";
    transport::SubscriberPtr sub = node->Subscribe(txTopic,
        &DepthCameraSensor_TEST::TxMsg, this);

    // Make sure that the sensor is updated and some messages are published
    for (int i = 0; i < 10; ++i)
    {
      common::Time::MSleep(100);
    }

    std::lock_guard<std::mutex> lock(this->reflectanceTopicMutex);
    EXPECT_FALSE(this->reflectanceTopicReceivedMsg);
  }
}

/////////////////////////////////////////////////
TEST_F(DepthCameraSensor_TEST, TestCreateDepthCameraSensor)
{
  TestCreateDepthCameraSensor();
}

/////////////////////////////////////////////////
TEST_F(DepthCameraSensor_TEST, TestDepthCameraReflectance)
{
  TestDepthCameraReflectance();
}

/////////////////////////////////////////////////
TEST_F(DepthCameraSensor_TEST, TestDepthCameraNormals)
{
  TestDepthCameraNormals();
}

/////////////////////////////////////////////////
TEST_F(DepthCameraSensor_TEST, TestReflectanceTopic)
{
  TestReflectanceTopic();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
