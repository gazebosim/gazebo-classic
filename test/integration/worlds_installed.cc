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
#include <boost/algorithm/string/find.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <gtest/gtest.h>

#include "test_config.h"

/////////////////////////////////////////////////
std::string customExec(std::string _cmd)
{
  _cmd += " 2>&1";
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
TEST(WorldsInstalled, checkWorlds)
{
  // Setup the path containing the worlds.
  boost::filesystem::path worldsPath(PROJECT_SOURCE_PATH);
  worldsPath /= "worlds";

  if (!boost::filesystem::exists(worldsPath) ||
      !boost::filesystem::is_directory(worldsPath))
  {
    std::cerr << "The worlds directory [" << worldsPath
              << "] does not exist or it's not a directory." << std::endl;
    FAIL();
  }

  // Iterate over the list of world files.
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator dir_itr(worldsPath);
          dir_itr != end_iter; ++dir_itr )
  {
    if (dir_itr->path().filename().extension() == ".world")
    {
      std::string cmd = "gz sdf --check " + dir_itr->path().string();
      std::string result = customExec(cmd);

      bool success = boost::algorithm::find_first(result, "Check complete");
      EXPECT_TRUE(success);
      if (!success)
      {
        std::cerr << "result: " << result << std::endl;
        std::cerr << "World file [" << dir_itr->path()
                  << "] is going to be installed but it's not SDF compliant.\n";
      }
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
