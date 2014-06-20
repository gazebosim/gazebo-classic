/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "test/util.hh"

const int g_messageRepeat = 4;

class Console_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test Console::Init and Console::Log
TEST_F(Console_TEST, InitAndLog)
{
  // Log the string
  std::string logString = "this is a test";

  gzlog << logString << std::endl;
  EXPECT_TRUE(this->GetLogContent().find(logString) != std::string::npos);
}

//////////////////////////////////////////////////
/// \brief Test Console::Log with \n characters
TEST_F(Console_TEST, LogSlashN)
{
  std::string logString = "this is a log test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzlog << logString << " _n__ " << i << '\n';
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::Log with std::endl
TEST_F(Console_TEST, LogStdEndl)
{
  std::string logString = "this is a log test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzlog << logString << " endl " << i << std::endl;
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorWarn with \n characters
TEST_F(Console_TEST, ColorWarnSlashN)
{
  std::string logString = "this is a warning test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzwarn << logString << " _n__ " << i << '\n';
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorWarn with std::endl
TEST_F(Console_TEST, ColorWarnStdEndl)
{
  std::string logString = "this is a warning test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzwarn << logString << " endl " << i << std::endl;
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorDbg with \n characters
TEST_F(Console_TEST, ColorDbgSlashN)
{
  std::string logString = "this is a dbg test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzdbg << logString << " _n__ " << i << '\n';
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorDbg with std::endl
TEST_F(Console_TEST, ColorDbgStdEndl)
{
  std::string logString = "this is a dbg test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzdbg << logString << " endl " << i << std::endl;
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorMsg with \n characters
TEST_F(Console_TEST, ColorMsgSlashN)
{
  std::string logString = "this is a msg test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzmsg << logString << " _n__ " << i << '\n';
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorMsg with std::endl
TEST_F(Console_TEST, ColorMsgStdEndl)
{
  std::string logString = "this is a msg test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzmsg << logString << " endl " << i << std::endl;
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorErr with \n characters
TEST_F(Console_TEST, ColorErrSlashN)
{
  std::string logString = "this is an error test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzerr << logString << " _n__ " << i << '\n';
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " _n__ " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

//////////////////////////////////////////////////
/// \brief Test Console::ColorErr with std::endl
TEST_F(Console_TEST, ColorErrStdEndl)
{
  std::string logString = "this is an error test";

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    gzerr << logString << " endl " << i << std::endl;
  }

  std::string logContent = this->GetLogContent();

  for (int i = 0; i < g_messageRepeat; ++i)
  {
    std::ostringstream stream;
    stream << logString << " endl " << i;
    EXPECT_TRUE(logContent.find(stream.str()) != std::string::npos);
  }
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorMsg
TEST_F(Console_TEST, ColorMsg)
{
  std::string logString = "this is a msg test";

  gzmsg << logString << std::endl;

  std::string logContent = this->GetLogContent();

  EXPECT_TRUE(logContent.find(logString) != std::string::npos);
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorErr
TEST_F(Console_TEST, ColorErr)
{
  std::string logString = "this is an error test";

  gzerr << logString << std::endl;

  std::string logContent = this->GetLogContent();

  EXPECT_TRUE(logContent.find(logString) != std::string::npos);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
