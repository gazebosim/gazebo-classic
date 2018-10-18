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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"

#define DEPTH_TOL 1e-4
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class DepthCameraSensorTest : public ServerFixture
{
  /// \brief Test placing a box in front of a depth camera
  public: void DepthUnitBox();

  /// \brief Depth camera image callback
  /// \param[in] _msg Message with image data containing raw depth values.
  public: void OnImage(ConstImageStampedPtr &_msg);

  /// \brief Counter for the numbder of image messages received.
  public: unsigned int depthCount;

  /// \brief Depth data buffer.
  public: float *depthBuffer;
};

/////////////////////////////////////////////////
void DepthCameraSensorTest::OnImage(ConstImageStampedPtr &_msg)
{
  // cppchecker recommends sizeof(varname)
  float f;
  unsigned int depthSamples = _msg->image().width() *_msg->image().height();
  unsigned int depthBufferSize = depthSamples * sizeof(f);
  if (!this->depthBuffer)
    this->depthBuffer = new float[depthSamples];
  memcpy(this->depthBuffer, _msg->image().data().c_str(), depthBufferSize);
  this->depthCount++;
}

/////////////////////////////////////////////////
void DepthCameraSensorTest::DepthUnitBox()
{
  // Test depth sensor with a unit box in the world.
  this->Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run depth camera test\n";
    return;
  }

  std::string modelName = "depth_model";
  std::string depthSensorName = "depth_sensor";
  double imgWidth = 320;
  double imgHeight = 240;
  double rate = 20;
  double far = 10;
  double near = 0.1;
  ignition::math::Pose3d testPose(0, 0, 0.1, 0, 0, 0);

  SpawnDepthCameraSensor(modelName, depthSensorName, testPose.Pos(),
      testPose.Rot().Euler(), imgWidth, imgHeight, rate, near, far);

  std::string box01 = "box_01";

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->GetPhysicsEngine()->SetGravity(ignition::math::Vector3d(0, 0, 0));

  // box in front of depth sensor
  ignition::math::Pose3d box01Pose(3, 0, 0.50, 0, 0, 0);

  SpawnBox(box01, ignition::math::Vector3d(1, 1, 1), box01Pose.Pos(),
      box01Pose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(depthSensorName);
  sensors::DepthCameraSensorPtr depthSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(depthSensor != NULL);
  depthSensor->SetActive(true);
  EXPECT_TRUE(depthSensor->IsActive());

  // Verify depth sensor depth readings
  // listen to new depth image frames
  this->depthBuffer = NULL;
  this->depthCount = 0;

  // Initialize gazebo transport layer
  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string depthCameraTopic = depthSensor->Topic();
  EXPECT_TRUE(!depthCameraTopic.empty());

  transport::SubscriberPtr sub = node->Subscribe(depthCameraTopic,
      &DepthCameraSensorTest::OnImage, this);

  // wait for a few depth images
  int i = 0;
  while (this->depthCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  int midWidth = depthSensor->ImageWidth() * 0.5;
  int midHeight = depthSensor->ImageHeight() * 0.5;
  int mid = midHeight * depthSensor->ImageWidth() + midWidth -1;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize * 0.5;

  // depth sensor should see box in the middle of the image
  EXPECT_NEAR(this->depthBuffer[mid], expectedRangeAtMidPoint, DEPTH_TOL);

  // the left and right side of the depth frame should be inf
  int left = midHeight * depthSensor->ImageWidth();
  EXPECT_DOUBLE_EQ(this->depthBuffer[left], IGN_DBL_INF);
  int right = (midHeight+1) * depthSensor->ImageWidth() - 1;
  EXPECT_DOUBLE_EQ(this->depthBuffer[right], IGN_DBL_INF);

  // move the box out of the range
  world->GetModel(box01)->SetWorldPose(
      ignition::math::Pose3d(far + unitBoxSize, 0, 0, 0, 0, 0));

  // wait for a few depth images
  i = 0;
  this->depthCount = 0;
  while (this->depthCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // the sensor should not be able to see the box any more
  EXPECT_DOUBLE_EQ(this->depthBuffer[mid], IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(this->depthBuffer[left], IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(this->depthBuffer[right], IGN_DBL_INF);

  // move the box closer than near clipping plane
  world->GetModel(box01)->SetWorldPose(
      ignition::math::Pose3d((near * 0.9) + unitBoxSize * 0.5, 0, 0, 0, 0, 0));

  // wait for a few depth images
  i = 0;
  this->depthCount = 0;
  while (this->depthCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // the box should be too close to get valid depth values
  // seems like EXPECT_DOUBLE_EQ does not work with -inf values
  EXPECT_TRUE(std::isinf(this->depthBuffer[mid]));
  EXPECT_TRUE(std::isinf(this->depthBuffer[left]));
  EXPECT_TRUE(std::isinf(this->depthBuffer[right]));

  if (this->depthBuffer)
    delete [] this->depthBuffer;
}

/////////////////////////////////////////////////
/// \brief Test depth camera sensor depth values
TEST_F(DepthCameraSensorTest, DepthUnitBox)
{
  DepthUnitBox();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
