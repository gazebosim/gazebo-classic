/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <condition_variable>
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class Sensor_TEST : public ServerFixture
{
};

std::condition_variable g_hokuyoCountCondition;
std::condition_variable g_imuCountCondition;

// global variable and callback for tracking hokuyo sensor messages
unsigned int g_hokuyoMsgCount;
void ReceiveHokuyoMsg(ConstLaserScanStampedPtr &/*_msg*/)
{
  g_hokuyoMsgCount++;
  if (g_hokuyoMsgCount >= 20)
    g_hokuyoCountCondition.notify_one();
}

// global variable and callback for tracking imu sensor messages
unsigned int g_imuMsgCount;
void ReceiveImuMsg(ConstLaserScanStampedPtr &/*_msg*/)
{
  g_imuMsgCount++;
  if (g_imuMsgCount >= 20)
    g_imuCountCondition.notify_one();
}

/////////////////////////////////////////////////
/// \brief Test that sensors will continue to update after Reset World
///        See bitbucket issue #236 for more background.
TEST_F(Sensor_TEST, UpdateAfterReset)
{
  // Load in a world with lasers
  Load("worlds/ray_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  unsigned int i;
  double updateRateHokuyo, updateRateImu, now, then;

  // get the sensor manager
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  EXPECT_TRUE(mgr->SensorsInitialized());

  // get the hokuyo sensor
  sensors::SensorPtr sensor;
  sensor = mgr->GetSensor("default::hokuyo::link::laser");
  ASSERT_TRUE(sensor != nullptr);

  sensors::SensorPtr imuSensor;
  imuSensor = mgr->GetSensor("default::box_model::box_link::box_imu_sensor");
  ASSERT_TRUE(imuSensor != nullptr);

  // set update rate to 30 Hz
  updateRateHokuyo = 30.0;
  updateRateImu = 1000.0;
  sensor->SetUpdateRate(updateRateHokuyo);
  imuSensor->SetUpdateRate(updateRateImu);
  gzdbg << sensor->ScopedName() << " loaded with update rate of "
        << sensor->UpdateRate() << " Hz"
        << std::endl;
  gzdbg << imuSensor->ScopedName() << " loaded with update rate of "
        << imuSensor->UpdateRate() << " Hz"
        << std::endl;

  g_hokuyoMsgCount = 0;
  g_imuMsgCount = 0;

  // Subscribe to sensor messages
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr laserSub = node->Subscribe(
      "~/hokuyo/link/laser/scan", &ReceiveHokuyoMsg);
  transport::SubscriberPtr imuSub = node->Subscribe(
      "~/box_model/box_link/box_imu_sensor/imu", &ReceiveImuMsg);

  // Wait for messages to arrive
  {
    std::mutex countMutex;
    std::unique_lock<std::mutex> lock(countMutex);
    g_hokuyoCountCondition.wait(lock);
    g_imuCountCondition.wait(lock);
  }

  unsigned int hokuyoMsgCount = g_hokuyoMsgCount;
  unsigned int imuMsgCount = g_imuMsgCount;
  now = world->GetSimTime().Double();

  gzdbg << "counted " << hokuyoMsgCount << " hokuyo messages in "
        << now << " seconds\n";
  gzdbg << "counted " << imuMsgCount << " imu messages in "
        << now << " seconds\n";

  // Expect at least 50% of specified update rate
  EXPECT_GT(static_cast<double>(hokuyoMsgCount),
              updateRateHokuyo*now * 0.5);
  EXPECT_GT(static_cast<double>(imuMsgCount),
              updateRateImu*now * 0.5);

  // Wait another 1.5 seconds
  for (i = 0; i < 15; ++i)
    common::Time::MSleep(100);

  hokuyoMsgCount = g_hokuyoMsgCount;
  imuMsgCount = g_imuMsgCount;
  now = world->GetSimTime().Double();

  gzdbg << "counted " << hokuyoMsgCount << " hokuyo messages in "
        << now << " seconds\n";
  gzdbg << "counted " << imuMsgCount << " imu messages in "
        << now << " seconds\n";

  // Expect at least 50% of specified update rate
  EXPECT_GT(static_cast<double>(hokuyoMsgCount),
              updateRateHokuyo*now * 0.5);
  EXPECT_GT(static_cast<double>(imuMsgCount),
              updateRateImu*now * 0.5);

  // Send reset world message
  transport::PublisherPtr worldControlPub =
    node->Advertise<msgs::WorldControl>("~/world_control");
  {
    // Copied from MainWindow::OnResetWorld
    msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);
    worldControlPub->Publish(msg);
  }
  gzdbg << "sent reset world message\n";
  common::Time::MSleep(100);
  now = world->GetSimTime().Double();
  gzdbg << "world time is now " << now << '\n';
  EXPECT_LT(now, 0.12);

  // Count messages again for 2 second
  g_hokuyoMsgCount = 0;
  g_imuMsgCount = 0;
  for (i = 0; i < 20; ++i)
  {
    common::Time::MSleep(100);
  }
  hokuyoMsgCount = g_hokuyoMsgCount;
  imuMsgCount = g_imuMsgCount;
  now = world->GetSimTime().Double() - now;
  gzdbg << "counted " << hokuyoMsgCount << " hokuyo messages in "
        << now << " seconds\n";
  gzdbg << "counted " << imuMsgCount << " imu messages in "
        << now << " seconds\n";

  // Expect at least 50% of specified update rate
  // Note: this is where the failure documented in issue #236 occurs
  EXPECT_GT(static_cast<double>(hokuyoMsgCount),
              updateRateHokuyo*now * 0.5);
  EXPECT_GT(static_cast<double>(imuMsgCount),
              updateRateImu*now * 0.5);

  // Count messages again for 2 more seconds
  then = now;
  g_hokuyoMsgCount = 0;
  g_imuMsgCount = 0;
  for (i = 0; i < 20; ++i)
  {
    common::Time::MSleep(100);
  }
  hokuyoMsgCount = g_hokuyoMsgCount;
  imuMsgCount = g_imuMsgCount;
  now = world->GetSimTime().Double();
  gzdbg << "counted " << hokuyoMsgCount << " hokuyo messages in "
        << now << " seconds\n";
  gzdbg << "counted " << imuMsgCount << " imu messages in "
        << now << " seconds\n";

  // Expect at least 50% of specified update rate
  EXPECT_GT(static_cast<double>(hokuyoMsgCount),
              updateRateHokuyo*(now-then) * 0.5);
  EXPECT_GT(static_cast<double>(imuMsgCount),
              updateRateImu*(now-then) * 0.5);
}

/////////////////////////////////////////////////
/// \brief Set pose
TEST_F(Sensor_TEST, SetPose)
{
  sensors::Sensor sensor(gazebo::sensors::OTHER);
  sensor.SetPose(ignition::math::Pose3d(0, 1, 2, 3, 4, 5));
  EXPECT_EQ(sensor.Pose(), ignition::math::Pose3d(0, 1, 2, 3, 4, 5));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
