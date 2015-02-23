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

#include "gazebo/common/Console.hh"

/////////////////////////////////////////////////
/// \brief Test Console::Init and Console::Log
TEST(Console_TEST, InitAndLog)
{
  // We need to create the log directory if needed.
  boost::filesystem::path logDirectory(getenv("HOME"));
  logDirectory = logDirectory / ".gazebo";
  if (!boost::filesystem::exists(logDirectory))
    boost::filesystem::create_directories(logDirectory);

  // Initialize Console
  gazebo::common::Console::Instance()->Init("test.log");
  EXPECT_TRUE(getenv("HOME") != NULL);

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
    std::getline(ifs, loggedString);
    EXPECT_TRUE(loggedString.find(logString) != std::string::npos);
  }
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorMsg
TEST(Console_TEST, ColorMsg)
{
  std::ostream *stream;
  stream = &(gazebo::common::Console::Instance()->ColorMsg("label", 20));

  EXPECT_TRUE(stream != NULL);
  EXPECT_TRUE(stream->good());
}

/////////////////////////////////////////////////
/// \brief Test Console::ColorErr
TEST(Console_TEST, ColorErr)
{
  std::ostream *stream;
  stream = &(gazebo::common::Console::Instance()->ColorErr("label",
        "myfile", 10, 20));

  EXPECT_TRUE(stream != NULL);
  EXPECT_TRUE(stream->good());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
