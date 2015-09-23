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

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <string>
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/LogPlay.hh"
#include "test_config.h"
#include "test/util.hh"

class LogPlay_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test LogPlay Open.
TEST_F(LogPlay_TEST, Open)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();
  EXPECT_FALSE(player->IsOpen());

  // Open a file that does not exist.
  EXPECT_ANY_THROW(player->Open("non-existing-file"));
  EXPECT_FALSE(player->IsOpen());

  boost::filesystem::path logFilePath(TEST_PATH);

  // Open a file that is a directory.
  EXPECT_ANY_THROW(player->Open(logFilePath.string()));
  EXPECT_FALSE(player->IsOpen());

  logFilePath = TEST_PATH / boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("invalidHeader.log");

  // Open a malformed log file (incorrect header).
  EXPECT_ANY_THROW(player->Open(logFilePath.string()));
  EXPECT_FALSE(player->IsOpen());

  logFilePath = TEST_PATH / boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  // Open a correct log file.
  EXPECT_NO_THROW(player->Open(logFilePath.string()));
  EXPECT_TRUE(player->IsOpen());
}

/////////////////////////////////////////////////
/// \brief Test LogPlay accessors.
TEST_F(LogPlay_TEST, Accessors)
{
  gazebo::common::Time expectedStartTime(28, 457000000);
  gazebo::common::Time expectedEndTime(31, 745000000);
  std::ostringstream expectedHeader;

  expectedHeader << "<?xml version='1.0'?>\n"
                 <<   "<gazebo_log>\n"
                 <<     "<header>\n"
                 <<       "<log_version>1.0</log_version>\n"
                 <<       "<gazebo_version>6.0.0</gazebo_version>\n"
                 <<       "<rand_seed>27838</rand_seed>\n"
                 <<       "<log_start>" << expectedStartTime << "</log_start>\n"
                 <<       "<log_end>" << expectedEndTime << "</log_end>\n"
                 <<     "</header>\n";

  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  // Test the accessors.
  EXPECT_EQ(player->GetLogVersion(), "1.0");
  EXPECT_EQ(player->GetGazeboVersion(), "6.0.0");
  EXPECT_EQ(player->GetRandSeed(), 27838u);
  EXPECT_EQ(player->GetLogStartTime(), expectedStartTime);
  EXPECT_EQ(player->GetLogEndTime(), expectedEndTime);
  EXPECT_EQ(player->GetFilename(), "state.log");
  EXPECT_EQ(player->GetFullPathFilename(), logFilePath.string());
  EXPECT_EQ(player->GetFileSize(), 341608u);
  EXPECT_EQ(player->GetEncoding(), "zlib");
  EXPECT_EQ(player->GetHeader(), expectedHeader.str());
  EXPECT_EQ(player->GetChunkCount(), 5u);
  EXPECT_FALSE(player->HasIterations());
  EXPECT_EQ(player->GetInitialIterations(), 0u);

  std::string chunk;
  EXPECT_TRUE(player->GetChunk(0, chunk));

  // Open a correct log file including <iterations>.
  logFilePath = TEST_PATH;
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state2.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));
  EXPECT_TRUE(player->HasIterations());
  EXPECT_EQ(player->GetInitialIterations(), 23700u);
}

/////////////////////////////////////////////////
/// \brief Test LogPlay chunks.
TEST_F(LogPlay_TEST, Chunks)
{
  std::string chunk;

  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  // Verify the content of chunk #0.
  player->GetChunk(0, chunk);
  std::string shasum = gazebo::common::get_sha1<std::string>(chunk);
  EXPECT_EQ(shasum, "aa227eee0554b8ace3a033e90b4f0c247909db33");

  // Make sure that the chunks returned are not empty.
  for (unsigned int i = 0; i < player->GetChunkCount(); ++i)
  {
    EXPECT_TRUE(player->GetChunk(i, chunk));
    EXPECT_TRUE(!chunk.empty());
  }

  // Try incorrect chunk indexes.
  EXPECT_FALSE(player->GetChunk(-1, chunk));
  EXPECT_FALSE(player->GetChunk(player->GetChunkCount(), chunk));
}

/////////////////////////////////////////////////
/// \brief Test Rewind().
TEST_F(LogPlay_TEST, Rewind)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  // Read the first entry in the log file.
  std::string firstEntry;
  // Consume the first chunk because it does not have <state>
  // ToDo: Fix this.
  EXPECT_TRUE(player->Step(firstEntry));
  // Read the first world state.
  EXPECT_TRUE(player->Step(firstEntry));

  // Step a few more times.
  std::string logEntry;
  for (int i = 0; i < 5; ++i)
    EXPECT_TRUE(player->Step(logEntry));

  // Rewind and read the first entry again.
  EXPECT_TRUE(player->Rewind());
  std::string entry;
  EXPECT_TRUE(player->Step(entry));
  EXPECT_EQ(entry, firstEntry);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
