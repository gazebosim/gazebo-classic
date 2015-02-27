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

#include <boost/filesystem.hpp>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"

using namespace gazebo;
class StateLogTest : public ServerFixture
{
};

double g_pr2LGripperXStart = -1;
double g_pr2LGripperXEnd = -1;
int g_msgCount = 0;

/////////////////////////////////////////////////
// Pose callback. We are just getting one link from the pr2 for simplicity.
void onPoseInfo(ConstPose_VPtr &_msg)
{
  for (int i = 0; i < _msg->pose_size(); ++i)
  {
    if (_msg->pose(i).name() == "pr2::l_gripper_r_parallel_link")
    {
      if (g_pr2LGripperXStart < 0)
        g_pr2LGripperXStart = _msg->pose(i).position().x();

      g_pr2LGripperXEnd = _msg->pose(i).position().x();
    }
  }

  ++g_msgCount;
}

/////////////////////////////////////////////////
/// Record a log file
TEST(StateLogTest, PR2Record)
{
  common::SystemPaths *paths = common::SystemPaths::Instance();
  custom_exec(("gzserver -r --record_path " + paths->GetDefaultTestPath()
    + " --iters 1000 --seed 12345 worlds/pr2.world").c_str());

  boost::filesystem::path path = paths->GetDefaultTestPath() + "/state.log";
  EXPECT_TRUE(boost::filesystem::exists(path) != false);
}

/////////////////////////////////////////////////
// Playback a log file
TEST(StateLogTest, PR2PlaybackZipped)
{
  // Cleanup...not the best
  custom_exec("killall -9 gzserver");

  // Run playback
  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::thread *play = new boost::thread(boost::bind(&custom_exec,
    ("gzserver -u -p " + paths->GetDefaultTestPath() "/state.log").c_str()));

  // Setup transportation
  gazebo::transport::init();
  gazebo::transport::run();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create a publisher to unpause gzserver
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::WorldControl>(
        "/gazebo/default/world_control");
  pub->WaitForConnection();

  // Subscribe to pose info
  gazebo::transport::SubscriberPtr sub = node->Subscribe(
      "/gazebo/default/pose/info", &onPoseInfo);

  // Unpause gzserver
  gazebo::msgs::WorldControl msg;
  msg.set_pause(false);
  pub->Publish(msg);

  int i = 0;
  // Wait for messages to arrive
  while (g_msgCount < 120 && i < 100)
  {
    gzdbg << "Pose Msg Count = " << g_msgCount  << std::endl;
    gazebo::common::Time::MSleep(100);
    ++i;
  }

  // Kill gserver
  play->interrupt();
  delete play;

  // Cleanup...not the best
  custom_exec("killall -9 gzserver");

  gazebo::transport::fini();

  if (g_msgCount >= 120)
  {
    // Make sure the values are correct.
    EXPECT_NEAR(g_pr2LGripperXStart, 0.83, 0.05);
    EXPECT_NEAR(g_pr2LGripperXEnd, 0.76, 0.05);
  }
}

/////////////////////////////////////////////////
// Playback a log file
TEST(StateLogTest, PR2PlaybackTxt)
{
  // Cleanup...not the best
  custom_exec("killall -9 gzserver");

  g_pr2LGripperXStart = -1;
  g_pr2LGripperXEnd = -1;
  g_msgCount = 0;

  // Convert the zipped state to txt and set a Hz filter.
  common::SystemPaths *paths = common::SystemPaths::Instance();
  custom_exec(("gzlog echo " + paths->GetDefaultTestPath() + "/state.log" +
    " -z 30 > " + paths->GetDefaultTestPath() + "/state_txt.log").c_str());

  // Run playback
  boost::thread *play = new boost::thread(boost::bind(&custom_exec,
    ("gzserver -u -p " + paths->GetDefaultTestPath() "/state.log").c_str()));

  // Setup transportation
  gazebo::transport::init();
  gazebo::transport::run();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create a publisher to unpause gzserver
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::WorldControl>(
        "/gazebo/default/world_control");
  pub->WaitForConnection();

  // Subscribe to pose info
  gazebo::transport::SubscriberPtr sub = node->Subscribe(
      "/gazebo/default/pose/info", &onPoseInfo);

  // Unpause gzserver
  gazebo::msgs::WorldControl msg;
  msg.set_pause(false);
  pub->Publish(msg);

  int i = 0;
  // Wait for messages to arrive
  while (g_msgCount < 120 && i < 100)
  {
    gzdbg << "Pose Msg Count = " << g_msgCount  << std::endl;
    gazebo::common::Time::MSleep(100);
    ++i;
  }

  // Kill gserver
  play->interrupt();
  delete play;

  // Cleanup...not the best
  custom_exec("killall -9 gzserver");

  gazebo::transport::fini();

  if (g_msgCount > 100)
  {
  // Make sure the values are correct.
  EXPECT_NEAR(g_pr2LGripperXStart, 0.83, 0.05);
  EXPECT_NEAR(g_pr2LGripperXEnd, 0.76, 0.05);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Cleanup test directory and create a new one.
  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::filesystem::remove_all(paths->GetDefaultTestPath());
  boost::filesystem::create_directories(paths->GetDefaultTestPath());

  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  // Cleanup test directory.
  boost::filesystem::remove_all(paths->GetDefaultTestPath());

  return result;
}
