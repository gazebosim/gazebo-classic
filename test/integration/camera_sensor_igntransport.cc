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

#include <ignition/math/Rand.hh>
#include <ignition/transport/Node.hh>

#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/CameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class CameraSensorIgnTransport : public ServerFixture
{
};

std::mutex mutex;

int imageCount = 0;

/////////////////////////////////////////////////
void OnNewCameraFrame(const ignition::msgs::Image &/*_msg*/)
{
  std::lock_guard<std::mutex> lock(mutex);
  ++imageCount;
}

/////////////////////////////////////////////////
TEST_F(CameraSensorIgnTransport, WorldReset)
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

  // Create the ignition::transport node
  ignition::transport::Node node;
  imageCount = 0;

  // Subscribe to the camera topic
  node.Subscribe(camSensor->TopicIgn(), &OnNewCameraFrame);

  const int kTotalImages = 20;
  common::Timer timer;

  common::Time dt = timer.GetElapsed();
  timer.Reset();
  timer.Start();

  int i = 0;
  for (; i < 1000; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (imageCount >= kTotalImages || timer.GetElapsed().Double() >= 4)
        break;
    }

    common::Time::MSleep(10);
  }
  EXPECT_LT(i, 1000);

  dt = timer.GetElapsed();
  EXPECT_GE(imageCount, kTotalImages);
  EXPECT_GT(dt.Double(), 1.0);
  EXPECT_LT(dt.Double(), 3.0);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
