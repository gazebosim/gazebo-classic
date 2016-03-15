/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include <thread>
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
  EXPECT_EQ(player->LogVersion(), "1.0");
  EXPECT_EQ(player->GazeboVersion(), "6.0.0");
  EXPECT_EQ(player->RandSeed(), 27838u);
  EXPECT_EQ(player->LogStartTime(), expectedStartTime);
  EXPECT_EQ(player->LogEndTime(), expectedEndTime);
  EXPECT_EQ(player->Filename(), "state.log");
  EXPECT_EQ(player->FullPathFilename(), logFilePath.string());
  EXPECT_EQ(player->FileSize(), 341608u);
  EXPECT_EQ(player->Encoding(), "zlib");
  EXPECT_EQ(player->Header(), expectedHeader.str());
  EXPECT_EQ(player->ChunkCount(), 5u);
  EXPECT_FALSE(player->HasIterations());
  EXPECT_EQ(player->InitialIterations(), 0u);

  std::string chunk;
  EXPECT_TRUE(player->Chunk(0, chunk));

  // Open a correct log file including <iterations>.
  logFilePath = TEST_PATH;
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state2.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));
  EXPECT_TRUE(player->HasIterations());
  EXPECT_EQ(player->InitialIterations(), 23700u);
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
  player->Chunk(0, chunk);
  std::string shasum = gazebo::common::get_sha1<std::string>(chunk);
  EXPECT_EQ(shasum, "aa227eee0554b8ace3a033e90b4f0c247909db33");

  // Make sure that the chunks returned are not empty.
  for (unsigned int i = 0; i < player->ChunkCount(); ++i)
  {
    EXPECT_TRUE(player->Chunk(i, chunk));
    EXPECT_TRUE(!chunk.empty());
  }

  // Try incorrect chunk indexes.
  EXPECT_FALSE(player->Chunk(-1, chunk));
  EXPECT_FALSE(player->Chunk(player->ChunkCount(), chunk));
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
  // Consume the first frame because it does not have <state>
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
/// \brief Test Forward().
TEST_F(LogPlay_TEST, Forward)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  EXPECT_TRUE(player->Forward());

  // Read the last frame.
  std::string frame;
  player->StepBack(frame);

  std::string expectedShashum = "961cf9dcd38c12f33a8b2f3a3a6fdb879b2faa98";
  std::string shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // Try again if I'm already at the end of the log.
  EXPECT_TRUE(player->Forward());
  player->StepBack(frame);
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // Jump to the beginning and then to the end.
  EXPECT_TRUE(player->Rewind());
  EXPECT_TRUE(player->Forward());
  player->StepBack(frame);
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);
}

/////////////////////////////////////////////////
/// \brief Test Step().
TEST_F(LogPlay_TEST, Step)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  std::string frame;
  // Consume the first frame because it does not have <state>
  // ToDo: Fix this.
  EXPECT_TRUE(player->Step(frame));
  // Read the first world state.
  EXPECT_TRUE(player->Step(frame));
  std::string expectedShashum = "0a61e946f14f7395a8bdb7974cb1e18c0d9e3d22";
  std::string shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  for (int i = 0; i < 1002; ++i)
    EXPECT_TRUE(player->Step(frame));

  // The last Step() should cause a transition to the next chunk.
  expectedShashum = "290a6f04c4c10867d1ed1697d09a7287be3e5500";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // You cannot Step() if you're at the end of the log file.
  player->Forward();
  EXPECT_FALSE(player->Step(frame));
}

/////////////////////////////////////////////////
/// \brief Test StepBack().
TEST_F(LogPlay_TEST, StepBack)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  EXPECT_TRUE(player->Forward());

  // This is the last frame (simulation time = 31.745).
  std::string frame;
  EXPECT_TRUE(player->StepBack(frame));
  std::string expectedShashum = "961cf9dcd38c12f33a8b2f3a3a6fdb879b2faa98";
  std::string shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // Read another frame (simulation time = 31.744).
  EXPECT_TRUE(player->StepBack(frame));
  expectedShashum = "c1cf8582d0cb6b628b89c22f91bb9573ee804bf6";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  for (int i = 0; i < 284; ++i)
    EXPECT_TRUE(player->StepBack(frame));

  // The last StepBack() should cause a transition to a different chunk.
  // Simulation time should be 31.460.
  expectedShashum = "043e1f2975619bf5b25aefab749d66f3aa510ef6";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // Rewind and try to StepBack().
  EXPECT_TRUE(player->Rewind());
  EXPECT_FALSE(player->StepBack(frame));
}

/////////////////////////////////////////////////
/// \brief Test multi-step().
TEST_F(LogPlay_TEST, MultiStep)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  player->Rewind();

  // Simulation time should be 28.465
  std::string frame;
  EXPECT_TRUE(player->Step(10, frame));
  std::string expectedShashum = "960543e7ac9cb2bcab5a7ee0bec314efb8d07e97";
  std::string shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // Simulation time should be 28.462
  EXPECT_TRUE(player->Step(-3, frame));
  expectedShashum = "83e173d438cd268ca475ea36350c914da25b51ca";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  EXPECT_TRUE(player->Step(-10, frame));

  // We should be at the beginning of the log file.
  expectedShashum = "30a3c4c09922a4fd15070c9eed84c89a3d1e8b53";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);

  // We can't execute a single step here.
  EXPECT_FALSE(player->Step(-2, frame));

  EXPECT_TRUE(player->Forward());
  // We can't execute a single step here.
  EXPECT_FALSE(player->Step(5, frame));
  EXPECT_TRUE(player->Step(-2, frame));

  // Simulation time should be 31.745
  EXPECT_TRUE(player->Step(10, frame));

  // We should be at the end of the log file.
  expectedShashum = "961cf9dcd38c12f33a8b2f3a3a6fdb879b2faa98";
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum);
}

/////////////////////////////////////////////////
/// \brief Test Seek().
TEST_F(LogPlay_TEST, Seek)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open a correct log file.
  boost::filesystem::path logFilePath(TEST_PATH);
  logFilePath /= boost::filesystem::path("logs");
  logFilePath /= boost::filesystem::path("state.log");

  EXPECT_NO_THROW(player->Open(logFilePath.string()));

  std::string expectedShashum1 = "a2af44bc561194dfeae9526c224d56bb332a4233";
  std::string expectedShashum2 = "113748a3c02575f514b27bc5b4307f621644ad41";
  std::string expectedShashum3 = "0a61e946f14f7395a8bdb7974cb1e18c0d9e3d22";
  std::string expectedShashum4 = "961cf9dcd38c12f33a8b2f3a3a6fdb879b2faa98";

  std::string frame;
  EXPECT_TRUE(player->Seek(common::Time(30.0)));
  EXPECT_TRUE(player->Step(frame));
  std::string shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum1);

  EXPECT_TRUE(player->Seek(common::Time(31.5)));
  EXPECT_TRUE(player->Step(frame));
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum2);

  EXPECT_TRUE(player->Seek(common::Time(30.0)));
  EXPECT_TRUE(player->Step(frame));
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum1);

  // Seek the to the beginning of the log.
  EXPECT_TRUE(player->Seek(common::Time(28.457)));
  EXPECT_TRUE(player->Step(frame));
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum3);

  // Seek the to the end of the log.
  EXPECT_TRUE(player->Seek(common::Time(31.745)));
  EXPECT_TRUE(player->Step(frame));

  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum4);

  // Try to seek before the beginning of the log.
  EXPECT_TRUE(player->Seek(common::Time(25.0)));
  EXPECT_TRUE(player->Step(frame));
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum3);

  // Try to seek after the end of the log.
  EXPECT_TRUE(player->Seek(common::Time(35.0)));
  EXPECT_TRUE(player->Step(frame));
  shasum = gazebo::common::get_sha1<std::string>(frame);
  EXPECT_EQ(shasum, expectedShashum4);
}

/////////////////////////////////////////////////
/// \brief Test reading a log file that is missing the closing </gazebo_log>
/// tag
TEST_F(LogPlay_TEST, NoEndTag)
{
  // \todo Make temporary files work in windows.
#ifndef _WIN32
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();

  // Open the log file that is missing the end tag
  std::ifstream srcFile(std::string(TEST_PATH) + "/logs/state_no_end.log",
      std::ios::binary);
  ASSERT_TRUE(srcFile.good());

  // Create a temporary test file.
  std::ostringstream stream;
  stream << "/tmp/__gz_log_test" << std::this_thread::get_id();

  std::string tmpFilename = stream.str();
  std::ofstream destFile(tmpFilename, std::ios::binary);
  ASSERT_TRUE(destFile.good());

  // Copy source to a temporary file so that we can modify it.
  destFile << srcFile.rdbuf();
  destFile.close();

  // Make sure we can read the log file
  EXPECT_NO_THROW(player->Open(tmpFilename));

  // Open the temporary file, which should now have the proper end tag
  std::ifstream inFile(tmpFilename);

  std::string endTag = "</gazebo_log>";

  // Back up the length of the closing tag.
  int len = -1 - static_cast<int>(endTag.length());
  inFile.seekg(len, std::ios::end);

  // Get the last line
  std::string lastLine;
  std::getline(inFile, lastLine);
  inFile.close();

  // Remove the temp file
  std::remove(tmpFilename.c_str());

  // Check that the log file now has the closing end tag
  EXPECT_EQ(lastLine, endTag);
#endif
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
