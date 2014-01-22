/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include <stdlib.h>

#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"

const int g_messageRepeat = 4;

/////////////////////////////////////////////////
/// \brief Test Console::Init and Console::Log
TEST(Console_TEST, InitAndLog)
{
  EXPECT_TRUE(getenv("HOME") != NULL);

  // We need to create the log directory if needed.
  boost::filesystem::path logDirectory(getenv("HOME"));
  logDirectory = logDirectory / ".gazebo";
  if (!boost::filesystem::exists(logDirectory))
    boost::filesystem::create_directories(logDirectory);

  // Initialize Console
  gzLogInit("test.log");

  // Make sure that the log file has been created
  boost::filesystem::path testLog;
  testLog = logDirectory / "test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  // Test Console::Log
  {
    std::string logString = "this is a test";
    std::string loggedString;

    // Log the string
    gzlog << logString << std::endl;

    // Open the log file, and read back the string
    std::ifstream ifs(testLog.string().c_str(), std::ios::in);
    while (!ifs.eof())
    {
      std::string line;
      std::getline(ifs, line);
      loggedString += line;
    }

    EXPECT_TRUE(loggedString.find(logString) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::Log with \n characters
TEST(Console_TEST, LogSlashN)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a log test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzlog << logString << " _n__ " << i << '\n';
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::Log with std::endl
TEST(Console_TEST, LogStdEndl)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a log test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzlog << logString << " endl " << i << std::endl;
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorWarn with \n characters
TEST(Console_TEST, ColorWarnSlashN)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a warning test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzwarn << logString << " _n__ " << i << '\n';
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorWarn with std::endl
TEST(Console_TEST, ColorWarnStdEndl)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a warning test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzwarn << logString << " endl " << i << std::endl;
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorDbg with \n characters
TEST(Console_TEST, ColorDbgSlashN)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a dbg test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzdbg << logString << " _n__ " << i << '\n';
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorDbg with std::endl
TEST(Console_TEST, ColorDbgStdEndl)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a dbg test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzdbg << logString << " endl " << i << std::endl;
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorMsg with \n characters
TEST(Console_TEST, ColorMsgSlashN)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a msg test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzmsg << logString << " _n__ " << i << '\n';
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorMsg with std::endl
TEST(Console_TEST, ColorMsgStdEndl)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a msg test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzmsg << logString << " endl " << i << std::endl;
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorErr with \n characters
TEST(Console_TEST, ColorErrSlashN)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is an error test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzerr << logString << " _n__ " << i << '\n';
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorErr with std::endl
TEST(Console_TEST, ColorErrStdEndl)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is an error test";
  std::string loggedString;

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzerr << logString << " endl " << i << std::endl;
  }

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(loggedString.find(stream.str()) != std::string::npos);
  }
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorMsg
TEST(Console_TEST, ColorMsg)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is a msg test";
  std::string loggedString;

  gzmsg << logString << std::endl;

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  EXPECT_TRUE(loggedString.find(logString) != std::string::npos);
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorErr
TEST(Console_TEST, ColorErr)
{
  // Initialize Console
  gzLogInit("test.log");
  gazebo::common::Console::SetQuiet(false);

  EXPECT_TRUE(getenv("HOME") != NULL);

  // Make sure that the log file has been created
  std::string logPath = getenv("HOME");
  boost::filesystem::path testLog(logPath);
  testLog = testLog / ".gazebo/test.log";
  EXPECT_TRUE(boost::filesystem::exists(testLog));

  std::string logString = "this is an error test";
  std::string loggedString;

  gzerr << logString << std::endl;

  // Open the log file, and read back the string
  std::ifstream ifs(testLog.string().c_str(), std::ios::in);
  while (!ifs.eof())
  {
    std::string line;
    std::getline(ifs, line);
    loggedString += line;
  }

  EXPECT_TRUE(loggedString.find(logString) != std::string::npos);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
