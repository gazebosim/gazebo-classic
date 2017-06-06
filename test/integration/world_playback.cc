/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <boost/filesystem.hpp>
#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/transport/TransportTypes.hh"

using namespace gazebo;

/// \brief Features to check in each log file.
/// The 1st element is the initial simulation time (frame #1).
/// The 2nd element is the simulation time at frame #2.
/// The 3rd element is the simulation time at frame #4.
/// The 4rd element is the last simulation time.
using FeaturesT = std::tuple<double, double, double, double>;

/// \brief List of log files for testing with their expected features.
/// The key of the map is the log file name.
/// The value is a tuple containing features to test (e.g.: sim times).
using LogFeatures_M = std::map<std::string, FeaturesT>;

// Contains the expected values for each log file under test.
LogFeatures_M logs =
{
  {"state.log",  FeaturesT {28.457, 28.457, 28.459, 31.745}},
  {"state2.log", FeaturesT {23.700, 23.700, 23.702, 26.168}},
  {"state3.log", FeaturesT {13.130, 15.778, 18.376, 22.773}}
};

/// \brief Helper class that initializes each test.
class WorldPlaybackTest : public ServerFixture,
                          public testing::WithParamInterface<const char *>
{
  /// \brief Class constructor.
  public: WorldPlaybackTest()
  {
    this->logName = std::string(this->GetParam());
    boost::filesystem::path logPath = TEST_PATH;
    logPath /= "logs/";
    logPath /= this->logName;

    // Start the server with a log file and paused.
    this->LoadArgs("-u -p " + logPath.string());
    this->world = physics::get_world("default");

    // Prepare the transport.
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    this->logPlaybackPub =
      this->node->Advertise<msgs::LogPlaybackControl>("~/playback_control");

    // Read expected values.
    this->features = logs[this->logName];

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  /// \brief Gazebo transport node.
  public: transport::NodePtr node;

  /// \brief Pointer to the world.
  public: physics::WorldPtr world;

  /// \brief Publisher to control the playback of the world via messages.
  public: transport::PublisherPtr logPlaybackPub;

  /// \brief The name of the log file used for testing.
  public: std::string logName;

  /// \brief Expected features for the current log under test.
  public: FeaturesT features;
};

/////////////////////////////////////////////////
/// \brief Check "pause".
TEST_P(WorldPlaybackTest, Pause)
{
  ASSERT_TRUE(this->world != NULL);
  EXPECT_TRUE(this->world->IsPaused());

  // Send a message to unpause the world.
  msgs::LogPlaybackControl msg;
  msg.set_pause(false);
  this->logPlaybackPub->Publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_TRUE(!this->world->IsPaused());

  // Send a message to pause the world.
  msg.set_pause(true);
  this->logPlaybackPub->Publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(this->world->IsPaused());
}

/////////////////////////////////////////////////
/// \brief Check "multi_step".
TEST_P(WorldPlaybackTest, Step)
{
  ASSERT_TRUE(this->world != NULL);
  msgs::LogPlaybackControl msg;
  common::Time expectedSimTime;

  // Rewind the world.
  expectedSimTime.Set(std::get<0>(this->features));
  msg.set_rewind(true);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 10, 20);

  // Step +1
  msg.Clear();
  expectedSimTime.Set(std::get<1>(this->features));
  msg.set_multi_step(1);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 10, 20);
  EXPECT_EQ(world->GetSimTime(), expectedSimTime);

  // Step -1
  msg.Clear();
  expectedSimTime.Set(std::get<0>(this->features));
  msg.set_multi_step(-1);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 10, 20);
  EXPECT_EQ(world->GetSimTime(), expectedSimTime);

  // Step +3
  msg.Clear();
  expectedSimTime.Set(std::get<2>(this->features));
  msg.set_multi_step(3);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 10, 20);
  EXPECT_EQ(world->GetSimTime(), expectedSimTime);

  // Step -2
  msg.Clear();
  expectedSimTime.Set(std::get<1>(this->features));
  msg.set_multi_step(-2);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 10, 20);
  EXPECT_EQ(world->GetSimTime(), expectedSimTime);

  // Insane backwards jump.
  msg.Clear();
  expectedSimTime.Set(std::get<0>(this->features));
  msg.set_multi_step(-9999);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 20, 20);
  EXPECT_EQ(world->GetSimTime(), expectedSimTime);

  // Insane forward jump.
  msg.Clear();
  expectedSimTime.Set(std::get<3>(this->features));
  msg.set_multi_step(999999);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 100, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);
}

/////////////////////////////////////////////////
/// \brief Check "rewind".
TEST_P(WorldPlaybackTest, Rewind)
{
  ASSERT_TRUE(this->world != NULL);

  for (const auto steps : {0, 1, -99999, 99999, 10, 100})
  {
    // Step forward.
    world->Step(steps);

    // Rewind the world.
    msgs::LogPlaybackControl msg;
    common::Time expectedSimTime;

    expectedSimTime.Set(std::get<0>(this->features));
    msg.set_rewind(true);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilSimTime(expectedSimTime, 10, 20);
    EXPECT_EQ(world->GetSimTime(), expectedSimTime);
  }
}

/////////////////////////////////////////////////
/// \brief Check "forward".
TEST_P(WorldPlaybackTest, Forward)
{
  ASSERT_TRUE(this->world != NULL);

  for (const auto steps : {1, 10, 0, 100})
  {
    // Step forward.
    world->Step(steps);

    // Fast forward the world.
    msgs::LogPlaybackControl msg;
    common::Time expectedSimTime;

    expectedSimTime.Set(std::get<3>(this->features));
    msg.set_forward(true);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilSimTime(expectedSimTime, 50, 400);
    EXPECT_EQ(world->GetSimTime(), expectedSimTime);
  }
}

/////////////////////////////////////////////////
/// \brief Check "seek".
TEST_P(WorldPlaybackTest, Seek)
{
  ASSERT_TRUE(this->world != NULL);
  common::Time expectedSimTime;

  // Move the simulation to the time at frame #2.
  expectedSimTime.Set(std::get<1>(this->features));
  auto msgExpectedTime = msgs::Convert(expectedSimTime);
  msgs::LogPlaybackControl msg;
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 50, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);

  // Move the simulation to the time at last frame.
  expectedSimTime.Set(std::get<3>(this->features));
  msgExpectedTime = msgs::Convert(expectedSimTime);
  msg.Clear();
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 50, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);

  // Specify a time slightly before #4.
  expectedSimTime.Set(std::get<2>(this->features));
  msgExpectedTime = msgs::Convert(expectedSimTime - 0.000001);
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 50, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);

  // Specify a time before the initial time.
  expectedSimTime.Set(std::get<0>(this->features));
  msgExpectedTime = msgs::Convert(expectedSimTime - 1.0);
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 50, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);

  // Specify a time after the last frame.
  expectedSimTime.Set(std::get<3>(this->features));
  msgExpectedTime = msgs::Convert(expectedSimTime + 1.0);
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedSimTime, 50, 50);
  EXPECT_EQ(this->world->GetSimTime(), expectedSimTime);
}

// Test with different log files.
//   - state.log:  File without <iterations> in each frame.
//   - state2.log: File with <iterations> in each frame.
//   - state3.log: File recording a simulation with an object that didn't move
//                 much. Two consecutive frames don't have consecutive
//                 simulation "iterations".
INSTANTIATE_TEST_CASE_P(LogFiles,
                        WorldPlaybackTest,
                        ::testing::Values("state.log",
                                          "state2.log",
                                          "state3.log"));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
