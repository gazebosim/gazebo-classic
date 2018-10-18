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
/// The 1st element is the initial iteration.
/// The 2nd element is the final iteration.
/// The 3rd element is the initial simulation time.
/// The 4rd element is the final simulation time.
using FeaturesT = std::tuple<uint32_t, uint32_t, double, double>;

/// \brief List of log files for testing with their expected features.
/// The key of the map is the log file name.
/// The value is a tuple containing features to test (e.g.: initial iteration).
using LogFeatures_M = std::map<std::string, FeaturesT>;

// Contains the expected values for each log file under test.
LogFeatures_M logs =
{
  {"state.log",  FeaturesT {1u, 3290u, 28.457, 31.745}},
  {"state2.log", FeaturesT {23700u, 26168u, 23.700, 26.168}}
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
    this->expectedInitialIters = std::get<0>(logs[this->logName]);
    this->expectedFinalIters = std::get<1>(logs[this->logName]);
    this->expectedInitialTime.Set(std::get<2>(logs[this->logName]));
    this->expectedFinalTime.Set(std::get<3>(logs[this->logName]));
  }

  /// \brief Gazebo transport node.
  public: transport::NodePtr node;

  /// \brief Pointer to the world.
  public: physics::WorldPtr world;

  /// \brief Publisher to control the playback of the world via messages.
  public: transport::PublisherPtr logPlaybackPub;

  /// \brief The name of the log file used for testing.
  public: std::string logName;

  /// \brief Initial "iterations" value contained in the current log file.
  public: uint64_t expectedInitialIters;

  /// \brief Final "iterations" value contained in the current log file.
  public: uint64_t expectedFinalIters;

  /// \brief Initial simulation time contained in the current log file.
  public: common::Time expectedInitialTime;

  /// \brief Final simulation time contained in the current log file.
  public: common::Time expectedFinalTime;
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
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  this->world->Step(10);

  // Test multiple steps.
  // Note: If you test a big step size you should increase the waiting time.
  msgs::LogPlaybackControl msg;
  for (auto step : {0, 1, 20, -1, -20})
  {
    auto curIters = this->world->GetIterations();
    auto expectedIters = curIters + step;
    msg.Clear();
    msg.set_multi_step(step);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilIteration(expectedIters, 50, 20);
    EXPECT_EQ(world->GetIterations(), expectedIters);
  }

  // Insane backwards jump.
  msg.Clear();
  msg.set_multi_step(-9999999);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilIteration(this->expectedInitialIters, 10, 20);
  EXPECT_EQ(world->GetIterations(), this->expectedInitialIters);

  // Insane forward jump.
  msg.Clear();
  msg.set_multi_step(9999999);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilIteration(this->expectedFinalIters, 500, 50);
  EXPECT_EQ(this->world->GetIterations(), this->expectedFinalIters);
}

/////////////////////////////////////////////////
/// \brief Check "rewind".
TEST_P(WorldPlaybackTest, Rewind)
{
  ASSERT_TRUE(this->world != NULL);

  for (auto steps : {0, 1, -99999, 99999, 10, 100})
  {
    // Step forward.
    world->Step(steps);

    // Rewind the world.
    msgs::LogPlaybackControl msg;
    msg.set_rewind(true);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilIteration(this->expectedInitialIters, 10, 20);
    EXPECT_EQ(world->GetIterations(), this->expectedInitialIters);
  }
}

/////////////////////////////////////////////////
/// \brief Check "forward".
TEST_P(WorldPlaybackTest, Forward)
{
  ASSERT_TRUE(this->world != NULL);

  for (auto steps : {1, 10, 0, 100})
  {
    // Rewind the world.
    msgs::LogPlaybackControl msg;
    msg.set_rewind(true);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilIteration(this->expectedInitialIters, 10, 20);
    EXPECT_EQ(world->GetIterations(), this->expectedInitialIters);

    // Step forward the world.
    world->Step(steps);

    // Fast forward the world.
    msg.Clear();
    msg.set_forward(true);
    this->logPlaybackPub->Publish(msg);
    this->WaitUntilIteration(this->expectedFinalIters, 50, 400);
    EXPECT_EQ(world->GetIterations(), this->expectedFinalIters);
  }
}

/////////////////////////////////////////////////
/// \brief Check "seek".
TEST_P(WorldPlaybackTest, Seek)
{
  ASSERT_TRUE(this->world != NULL);

  this->world->Step(10);

  // Move the simulation 0.5 secs in the future.
  auto expectedTime = this->world->GetSimTime() + 0.5;
  auto msgExpectedTime = msgs::Convert(expectedTime);
  msgs::LogPlaybackControl msg;
  msg.mutable_seek()->set_sec(msgExpectedTime.sec());
  msg.mutable_seek()->set_nsec(msgExpectedTime.nsec());
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedTime, 50, 50);
  EXPECT_TRUE(this->world->GetSimTime() == expectedTime);

  // Try to seek to before the initial time.
  msg.Clear();
  expectedTime = this->expectedInitialTime;
  msg.mutable_seek()->set_sec(0);
  msg.mutable_seek()->set_nsec(0);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedTime, 50, 50);
  EXPECT_TRUE(this->world->GetSimTime() == expectedTime);

  // Try to seek after the final time.
  msg.Clear();
  expectedTime = this->expectedFinalTime;
  msg.mutable_seek()->set_sec(999999);
  msg.mutable_seek()->set_nsec(0);
  this->logPlaybackPub->Publish(msg);
  this->WaitUntilSimTime(expectedTime, 100, 100);
  EXPECT_TRUE(this->world->GetSimTime() == expectedTime);
}

// Test with different log files.
INSTANTIATE_TEST_CASE_P(LogFiles,
                        WorldPlaybackTest,
                        ::testing::Values("state.log", "state2.log"));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
