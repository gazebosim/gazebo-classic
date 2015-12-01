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

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class RenderingSensorTest : public ServerFixture
{
};


common::Time lastTimestamp;
std::vector<common::Time> gpuRayTimeStamps;
std::vector<common::Time> cam1TimeStamps;
std::vector<common::Time> cam2TimeStamps;

// Collects gpu ray sensor's data timestamps for the TimeStamp test
void OnReceiveGpuRayMsg(ConstLaserScanStampedPtr &_msg)
{
  gpuRayTimeStamps.push_back(
      common::Time(_msg->time().sec(), _msg->time().nsec()));
}

// Collects camera sensor's data timestamps for the TimeStamp test
void OnReceiveCamera1Msg(ConstImageStampedPtr &_msg)
{
  cam1TimeStamps.push_back(
      common::Time(_msg->time().sec(), _msg->time().nsec()));
}

// Collects camera sensor2's data timestamps for the TimeStamp test
void OnReceiveCamera2Msg(ConstImageStampedPtr &_msg)
{
  cam2TimeStamps.push_back(
      common::Time(_msg->time().sec(), _msg->time().nsec()));
}

/////////////////////////////////////////////////
/// \brief Create mulitiple rendering sensors so that there's some contention
/// for gpu resources. Verify delays do not cause duplicate timestamps
TEST_F(RenderingSensorTest, Timestamp)
{
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // Spawn multiple camera sensors with reasonably high resolution and
  // framerate. The long update time needed for these camera sensors cause
  // delays to other rendering sensors updates, resulting in the sensors trying
  // to catch up in order to achieve their target update rate. The test verifies
  // that this update strategy does not produce duplicate timestamped data and
  // are in the right order.
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 800;
  unsigned int height = 800;
  double updateRate = 30;
  math::Pose camPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, camPose.pos,
      camPose.rot.GetAsEuler(), width, height, updateRate);

  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  EXPECT_TRUE(sensor != NULL);

  sensors::CameraSensorPtr camSensor1 =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor1 != NULL);

  std::string modelName2 = "camera_model2";
  std::string cameraName2 = "camera_sensor2";
  math::Pose camPose2(
      math::Vector3(5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName2, cameraName2, camPose2.pos,
      camPose2.rot.GetAsEuler(), width, height, updateRate);

  sensors::SensorPtr sensor2 = sensors::get_sensor(cameraName2);
  EXPECT_TRUE(sensor2 != NULL);

  sensors::CameraSensorPtr camSensor2 =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);
  EXPECT_TRUE(camSensor2 != NULL);

  // spawn gpu ray sensor
  std::string modelName3 = "gpu_ray_model";
  std::string raySensorName = "gpu_ray_sensor";
  double hMinAngle = -2.0;
  double hMaxAngle = 2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 640;
  math::Pose testPose(math::Vector3(0, 0, 0.1),
      math::Quaternion(0, 0, 0));

  SpawnGpuRaySensor(modelName3, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  sensors::SensorPtr sensor3 = sensors::get_sensor(raySensorName);
  ASSERT_TRUE(sensor3 != NULL);

  sensors::GpuRaySensorPtr gpuRaySensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor3);
  EXPECT_TRUE(gpuRaySensor != NULL);

  camSensor1->SetActive(true);
  camSensor2->SetActive(true);
  gpuRaySensor->SetActive(true);

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();

  transport::SubscriberPtr cam1Sub = node->Subscribe(camSensor1->Topic(),
      &OnReceiveCamera1Msg);
  transport::SubscriberPtr cam2Sub = node->Subscribe(camSensor2->Topic(),
      &OnReceiveCamera2Msg);
  transport::SubscriberPtr gpuRaySub = node->Subscribe(gpuRaySensor->Topic(),
      &OnReceiveGpuRayMsg);

  unsigned int numTimestamps = 100;
  // wait for a few laser scans
  gpuRayTimeStamps.clear();
  cam1TimeStamps.clear();
  cam2TimeStamps.clear();
  int i = 0;
  while ((gpuRayTimeStamps.size() < numTimestamps ||
      cam1TimeStamps.size() < numTimestamps ||
      cam2TimeStamps.size() < numTimestamps) && i < 500)
  {
    common::Time::MSleep(100);
    i++;
  }
  ASSERT_LT(i, 500);

  // Verify that there are no duplicate timestamps
  for (unsigned int j = 0; j < numTimestamps - 1; ++j)
  {
    EXPECT_TRUE (gpuRayTimeStamps[j] < gpuRayTimeStamps[j+1]);
    EXPECT_TRUE (cam1TimeStamps[j] < cam1TimeStamps[j+1]);
    EXPECT_TRUE (cam2TimeStamps[j] < cam2TimeStamps[j+1]);
  }

  gpuRayTimeStamps.clear();
  cam1TimeStamps.clear();
  cam2TimeStamps.clear();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
