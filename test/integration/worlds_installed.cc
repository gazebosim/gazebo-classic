/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/find.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include <gtest/gtest.h>

#include "test_config.h"

const std::string BeginCheckSdf = "#BEGIN_CHECK_SDF";
const std::string EndCheckSdf   = "#END_CHECK_SDF";

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
bool hasSdfLabels(const std::string &_path)
{
  std::string line;
  bool beginLabelFound = false;
  bool endLabelFound = false;

  std::ifstream file(_path.c_str());
  if (!file.is_open())
    return false;

  while (getline(file, line))
  {
    if (line == BeginCheckSdf)
      beginLabelFound = true;
    else if (line == EndCheckSdf)
      endLabelFound = true;

    if (beginLabelFound && endLabelFound)
      break;
  }

  file.close();
  return beginLabelFound && endLabelFound;
}

/////////////////////////////////////////////////
TEST(WorldsInstalled, checkWorlds)
{
  // Setup some paths
  boost::filesystem::path worldsPath(PROJECT_SOURCE_PATH);
  worldsPath /= "worlds";

  boost::filesystem::path worldsCmakePath(worldsPath);
  worldsCmakePath /= "CMakeLists.txt";

  // Open the CMakeLists.txt containing the worlds to be installed.
  std::ifstream worldsCmakeFile(worldsCmakePath.c_str());
  ASSERT_TRUE(worldsCmakeFile.is_open());

  // Make sure that the SDF check labels exist in the CMakeLists.txt .
  if (!hasSdfLabels(worldsCmakePath.string()))
  {
    std::cerr << "Check [" << BeginCheckSdf << "] and [" << EndCheckSdf
              << "] labels in [" << worldsCmakePath << "]. Labels not found.\n";
    FAIL();
  }

  // Ignore all the lines until the BeginCheckSdf label.
  std::string line;
  while (getline(worldsCmakeFile, line))
    if (line == BeginCheckSdf)
      break;

  // Check the SDF of all the world files until EndCheckSdf label.
  while (getline(worldsCmakeFile, line))
  {
    if (line == EndCheckSdf)
      break;

    boost::algorithm::erase_all(line, " ");
    boost::filesystem::path worldPath = worldsPath / line;
    std::string cmd = "gzsdf check " + worldPath.string();
    std::string result = customExec(cmd);

    bool success = boost::algorithm::find_first(result, "Success");
    EXPECT_TRUE(success);
    if (!success)
      std::cerr << "World file [" << worldPath.string() << "] is going to be "
                << "installed but it is not SDF compliant." << std::endl;
  }

  worldsCmakeFile.close();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
