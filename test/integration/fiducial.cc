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
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

// this is the test fixture
class FiducialCameraTest : public ServerFixture
{
};

std::mutex g_mutex;
msgs::PosesStamped g_fiducialPoseMsg;
bool g_received = false;

/////////////////////////////////////////////////
void OnFiducial(ConstPosesStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_fiducialPoseMsg = *_msg.get();
  g_received = true;
}

/////////////////////////////////////////////////
TEST_F(FiducialCameraTest, Fiducial)
{
  this->Load("worlds/fiducial.world");

  // Get the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  std::set<std::string> fiducials;

  // Get the fiducial_01 model
  std::string fiducial01Name = "fiducial_01";
  physics::ModelPtr fiducial01 = world->GetModel(fiducial01Name);
  ASSERT_TRUE(fiducial01 != nullptr);
  fiducials.insert(fiducial01Name);

  // Get the fiducial_02 model
  std::string fiducial02Name = "fiducial_02";
  physics::ModelPtr fiducial02 = world->GetModel(fiducial02Name);
  ASSERT_TRUE(fiducial02 != nullptr);
  fiducials.insert(fiducial02Name);

  sensors::SensorPtr sensor = sensors::get_sensor("camera");
  EXPECT_TRUE(sensor != nullptr);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor != nullptr);

  // subscribe to fiducial camera topic
  std::string topicName = "~/";
  topicName += camSensor->ParentName() + "/" + camSensor->Name() + "/fiducial";

  size_t pos;
  while ((pos = topicName.find("::")) != std::string::npos)
    topicName = topicName.substr(0, pos) + "/" + topicName.substr(pos+2);

  auto fiducialSub = node->Subscribe(topicName, &OnFiducial);
  ASSERT_TRUE(fiducialSub != nullptr);

  // wait for a message
  int sleep = 0;
  int maxSleep = 20;
  bool received = false;
  while (!received && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(30);
    sleep++;

    std::lock_guard<std::mutex> lock(g_mutex);
    received = g_received;
    g_received = false;
  }
  EXPECT_TRUE(received);

  // check that both fiducials are detected
  std::set<std::string> fiducialsCopy;
  fiducialsCopy = fiducials;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    EXPECT_EQ(g_fiducialPoseMsg.pose_size(), 2);
    for (int i = 0; i < g_fiducialPoseMsg.pose_size(); ++i)
    {
      std::string fiducialName = g_fiducialPoseMsg.pose(i).name();
      EXPECT_TRUE(fiducialsCopy.find(fiducialName) != fiducialsCopy.end());
      fiducialsCopy.erase(fiducialName);
    }
  }

  // move fiducial_01 out of view
  fiducial01->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(10, 0, 0),
      ignition::math::Quaterniond::Identity));

  // wait for new fiducial message with a timestamp later than the time
  // fiducial_01 was moved
  common::Time poseUpdateTime = world->GetSimTime();
  sleep = 0;
  received = false;
  while (!received && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(30);
    sleep++;

    std::lock_guard<std::mutex> lock(g_mutex);
    common::Time timestamp = msgs::Convert(g_fiducialPoseMsg.time());
    received = g_received && timestamp > poseUpdateTime;
    g_received = false;
  }
  EXPECT_TRUE(received);

  // verify only fiducial_02 is in the view
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    EXPECT_EQ(g_fiducialPoseMsg.pose_size(), 1);
    EXPECT_EQ(g_fiducialPoseMsg.pose(0).name(), "fiducial_02");
  }

  // test occlusion by spawning a box over fiducial_02
  ignition::math::Vector3d boxPos(ignition::math::Vector3d(0, 0, 1) +
      fiducial02->GetWorldPose().pos.Ign());

  SpawnBox("test_box", ignition::math::Vector3d::One, boxPos,
      ignition::math::Vector3d::Zero);
  common::Time spawnBoxTime = world->GetSimTime();

  // wait for new fiducial message with a timestamp later than the time
  // test_box was spawned
  sleep = 0;
  received = false;
  while (!received && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(30);
    sleep++;

    std::lock_guard<std::mutex> lock(g_mutex);
    common::Time timestamp = msgs::Convert(g_fiducialPoseMsg.time());
    received = g_received && timestamp > spawnBoxTime;
    g_received = false;
  }
  EXPECT_TRUE(received);

  // verify no fiducials are detected
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    EXPECT_EQ(g_fiducialPoseMsg.pose_size(), 0);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
