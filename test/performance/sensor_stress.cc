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

#include <gtest/gtest.h>
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class SensorStress_TEST : public ServerFixture
{
};

boost::condition_variable g_countCondition;

// global variable and callback for tracking hokuyo sensor messages
unsigned int g_hokuyoMsgCount;
void ReceiveHokuyoMsg(ConstLaserScanStampedPtr &/*_msg*/)
{
  g_hokuyoMsgCount++;
  if (g_hokuyoMsgCount >= 20)
    g_countCondition.notify_one();
}

/////////////////////////////////////////////////
/// \brief Reset world a bunch of times and verify that no assertions happen
/// The assert "SensorManager.cc(479): Took negative time to update a sensor."
/// has been observed in Jenkins testing.
TEST_F(SensorStress_TEST, ResetWorldStressTest)
{
  // Load in a world with lasers
  Load("worlds/ray_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // get the sensor manager
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  EXPECT_TRUE(mgr->SensorsInitialized());

  // get the hokuyo sensor
  sensors::SensorPtr sensor;
  sensor = mgr->GetSensor("default::hokuyo::link::laser");
  ASSERT_TRUE(sensor != NULL);

  // set update rate to unlimited
  double updateRate = 0.0;
  sensor->SetUpdateRate(updateRate);
  gzdbg << sensor->ScopedName() << " loaded with update rate of "
        << sensor->UpdateRate() << " Hz\n";

  g_hokuyoMsgCount = 0;

  // Subscribe to hokuyo laser scan messages
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe(
      "~/hokuyo/link/laser/scan", &ReceiveHokuyoMsg);

  // Wait for messages to arrive
  {
    boost::mutex countMutex;
    boost::mutex::scoped_lock lock(countMutex);
    g_countCondition.wait(lock);
    gzdbg << "counted " << g_hokuyoMsgCount << " hokuyo messages\n";
  }

  EXPECT_GT(g_hokuyoMsgCount, 19u);

  // Send reset world message
  transport::PublisherPtr worldControlPub =
    node->Advertise<msgs::WorldControl>("~/world_control");

  // Copied from MainWindow::OnResetWorld
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  worldControlPub->Publish(msg);

  common::Time::MSleep(300);

  int i;
  for (i = 0; i < 20; ++i)
  {
    worldControlPub->Publish(msg);
    gzdbg << "counted " << g_hokuyoMsgCount << " hokuyo messages\n";
    common::Time::MSleep(200);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
