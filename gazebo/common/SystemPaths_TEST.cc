/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <string>
#include <vector>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SystemPaths.hh"
#include "test/util.hh"

using namespace gazebo;

class SystemPathsTest : public gazebo::testing::AutoLogFixture { };

TEST_F(SystemPathsTest, SystemPaths)
{
  std::vector<std::string> tmpstrings;
  std::string gazeboResourcePathBackup = "GAZEBO_RESOURCE_PATH=";
  std::string ogreResourcePathBackup = "OGRE_RESOURCE_PATH=";
  std::string pluginPathBackup = "GAZEBO_PLUGIN_PATH=";

  if (getenv("GAZEBO_RESOURCE_PATH"))
    gazeboResourcePathBackup += getenv("GAZEBO_RESOURCE_PATH");

  if (getenv("GAZEBO_RESOURCE_PATH"))
    ogreResourcePathBackup += getenv("GAZEBO_RESOURCE_PATH");

  if (getenv("GAZEBO_PLUGIN_PATH"))
    pluginPathBackup += getenv("GAZEBO_PLUGIN_PATH");

  putenv(const_cast<char*>("GAZEBO_LOG_PATH="));
  common::SystemPaths *paths = common::SystemPaths::Instance();

  paths->ClearGazeboPaths();
  paths->ClearOgrePaths();
  paths->ClearPluginPaths();

  std::string gzResourcePath = "GAZEBO_RESOURCE_PATH=" + paths->TmpPath() +
      "/resource:/test/me/now";
  putenv(const_cast<char*>(gzResourcePath.c_str()));
  const std::list<std::string> &pathList1 = paths->GetGazeboPaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList1.size());
  EXPECT_STREQ((paths->TmpPath() + "/resource").c_str(),
      pathList1.front().c_str());
  EXPECT_STREQ("/test/me/now", pathList1.back().c_str());

  std::string ogreResourcePath = "OGRE_RESOURCE_PATH=" + paths->TmpPath() +
      "/ogre:/test/ogre/now";
  putenv(const_cast<char*>(ogreResourcePath.c_str()));
  const std::list<std::string> &pathList2 = paths->GetOgrePaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList2.size());
  EXPECT_STREQ((paths->TmpPath() + "/ogre").c_str(),
      pathList2.front().c_str());
  EXPECT_STREQ("/test/ogre/now", pathList2.back().c_str());

  std::string gzPluginPath = "GAZEBO_PLUGIN_PATH=" + paths->TmpPath() +
      "/plugin:/test/plugin/now";
  putenv(const_cast<char*>(gzPluginPath.c_str()));
  const std::list<std::string> &pathList3 = paths->GetPluginPaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList3.size());
  EXPECT_STREQ((paths->TmpPath() + "/plugin").c_str(),
      pathList3.front().c_str());
  EXPECT_STREQ("/test/plugin/now", pathList3.back().c_str());

  EXPECT_STREQ("/worlds", paths->GetWorldPathExtension().c_str());

  paths->AddGazeboPaths("/gazebo/path:/other/gazebo");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetGazeboPaths().size());
  EXPECT_STREQ("/other/gazebo", paths->GetGazeboPaths().back().c_str());

  paths->AddPluginPaths("/plugin/path:/other/plugin");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetPluginPaths().size());
  EXPECT_STREQ("/other/plugin", paths->GetPluginPaths().back().c_str());

  paths->AddOgrePaths("/ogre/path:/other/ogre");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetOgrePaths().size());
  EXPECT_STREQ("/other/ogre", paths->GetOgrePaths().back().c_str());

  paths->ClearGazeboPaths();
  paths->ClearOgrePaths();
  paths->ClearPluginPaths();

  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetGazeboPaths().size());
  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetOgrePaths().size());
  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetPluginPaths().size());

  putenv(const_cast<char*>("GAZEBO_RESOURCE_PATH="));
  paths->ClearGazeboPaths();
  // In this case, we expect to get the compiled-in default
  tmpstrings = common::split(GAZEBO_RESOURCE_PATH, ":");
  EXPECT_EQ(tmpstrings.size(), paths->GetGazeboPaths().size());

  putenv(const_cast<char*>("OGRE_RESOURCE_PATH="));
  paths->ClearOgrePaths();
  // In this case, we expect to get the compiled-in default
  tmpstrings = common::split(OGRE_RESOURCE_PATH, ":");
  EXPECT_EQ(tmpstrings.size(), paths->GetOgrePaths().size());

  putenv(const_cast<char*>("GAZEBO_PLUGIN_PATH="));
  paths->ClearPluginPaths();
  // In this case, we expect to get the compiled-in default
  tmpstrings = common::split(GAZEBO_PLUGIN_PATH, ":");
  EXPECT_EQ(tmpstrings.size(), paths->GetPluginPaths().size());

  std::cout << "GAZEBO_RESOURCE_BACKUP[" << gazeboResourcePathBackup << "]\n";
  std::cout << "OGRE_RESOURCE_BACKUP[" << ogreResourcePathBackup << "]\n";
  std::cout << "GAZEBO_PLUGIN_BACKUP[" << ogreResourcePathBackup << "]\n";

  putenv(const_cast<char*>(gazeboResourcePathBackup.c_str()));
  putenv(const_cast<char*>(ogreResourcePathBackup.c_str()));
  putenv(const_cast<char*>(pluginPathBackup.c_str()));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
