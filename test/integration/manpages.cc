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
#include <string>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "test_config.h"

boost::filesystem::path g_mkRonn;
boost::filesystem::path g_gzRoff;
boost::filesystem::path g_toolBinPath;
boost::filesystem::path g_toolSrcPath;
boost::filesystem::path g_serverBinPath;
boost::filesystem::path g_serverSrcPath;
boost::filesystem::path g_guiBinPath;
boost::filesystem::path g_guiSrcPath;
std::string g_gazeboPATH;

/////////////////////////////////////////////////
std::string customExec(std::string _cmd)
{
  _cmd += " 2>/dev/null";
  FILE* pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
std::string readFile(const std::string &_filename)
{
  std::string result;
  std::string line;

  std::ifstream ronnFile(_filename.c_str(), std::ios::in);
  while (ronnFile)
  {
    std::getline(ronnFile, line);
    if (ronnFile)
      result += line + "\n";
  }

  return result;
}

/////////////////////////////////////////////////
void toolTest(const std::string &_command)
{
  std::string ronnOut = customExec(g_mkRonn.string()
      + " " + (g_toolBinPath / _command).string());

  boost::filesystem::path origRonnFilename =
    g_toolSrcPath / (_command + ".1.ronn");

  std::string origRonn = readFile(origRonnFilename.string());

  EXPECT_EQ(origRonn, ronnOut);
}

/////////////////////////////////////////////////
void serverTest(const std::string &_command)
{
  std::string ronnOut = customExec(g_gazeboPATH + " " + g_mkRonn.string()
      + " " + (g_serverBinPath / _command).string());

  boost::filesystem::path origRonnFilename =
    g_serverSrcPath / (_command + ".1.ronn");

  std::string origRonn = readFile(origRonnFilename.string());

  EXPECT_EQ(origRonn, ronnOut);
}

/////////////////////////////////////////////////
void guiTest(const std::string &_command)
{
  std::string ronnOut = customExec(g_mkRonn.string()
      + " " + (g_guiBinPath / _command).string());

  boost::filesystem::path origRonnFilename =
    g_guiSrcPath / (_command + ".1.ronn");

  std::string origRonn = readFile(origRonnFilename.string());

  EXPECT_EQ(origRonn, ronnOut);
}

/////////////////////////////////////////////////
TEST(ManTest, gzclient)
{
  guiTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gzserver)
{
  serverTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gazebo)
{
  serverTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gzfactory)
{
  toolTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gzlog)
{
  toolTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gzsdf)
{
  toolTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gzstats)
{
  toolTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gztopic)
{
  toolTest(::testing::UnitTest::GetInstance()->current_test_info()->name());
}

/////////////////////////////////////////////////
TEST(ManTest, gz)
{
  std::string command =
    ::testing::UnitTest::GetInstance()->current_test_info()->name();

  std::string roffOut = customExec(g_gzRoff.string()
      + " " + (g_toolBinPath / command).string());

  boost::filesystem::path origRoffFilename =
    g_toolSrcPath / (command + ".1.roff");

  std::string origRoff = readFile(origRoffFilename.string());

  EXPECT_EQ(origRoff, roffOut);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  g_mkRonn = PROJECT_SOURCE_PATH;
  g_mkRonn = g_mkRonn / "tools" / "make_ronn.py";

  g_gzRoff = PROJECT_SOURCE_PATH;
  g_gzRoff = g_gzRoff / "tools" / "gz_roff.py";

  g_toolBinPath = PROJECT_BINARY_PATH;
  g_toolBinPath = g_toolBinPath / "tools";

  g_toolSrcPath = PROJECT_SOURCE_PATH;
  g_toolSrcPath = g_toolSrcPath / "tools";

  g_serverBinPath = PROJECT_BINARY_PATH;
  g_serverBinPath = g_serverBinPath / "gazebo";

  g_serverSrcPath = PROJECT_SOURCE_PATH;
  g_serverSrcPath = g_serverSrcPath / "gazebo";

  g_guiBinPath = g_serverBinPath / "gui";
  g_guiSrcPath = g_serverSrcPath / "gui";

  g_gazeboPATH = std::string("PATH=") + g_serverBinPath.string() + ":"
      + g_guiBinPath.string() + ":$PATH";

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
