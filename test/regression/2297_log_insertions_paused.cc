/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef _WIN32
#include <unistd.h>
#else
#include <boost/filesystem.hpp>
#endif
#include <gazebo/util/LogRecord.hh>
#include <gazebo/util/LogPlay.hh>
#include <gazebo/test/ServerFixture.hh>

using namespace gazebo;

class Issue2297_Test : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue2297_Test, ModelInsertRecord)
{
#ifndef _WIN32
  // Create a temporary directory
  char dirTemplate[] ="/tmp/gazeboXXXXXX";
  std::string tmpDir = mkdtemp(dirTemplate);
#else
  boost::filesystem::path tmppath = boost::filesystem::temp_directory_path();
  std::string tmpDir = tmppath.string();
#endif

  // Initialize log recording
  util::LogRecord *recorder = util::LogRecord::Instance();
  ASSERT_TRUE(recorder != nullptr);
  recorder->Init("test");

  // Start gazebo paused.
  this->Load("worlds/empty.world", true);

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  recorder->Start("txt", tmpDir);
  std::string filename = recorder->Filename();

  std::string modelString =
      "<sdf version='1.6'>"
      "  <model name='box'>"
      "    <pose>0 0 10 0 0 0</pose>"
      "    <link name='link'>"
      "      <collision name='collision'>"
      "        <geometry><box><size>1 1 1</size></box></geometry>"
      "      </collision>"
      "      <visual name='visual'>"
      "        <geometry><box><size>1 1 1</size></box></geometry>"
      "      </visual>"
      "    </link>"
      "  </model>"
      "</sdf>";

  world->InsertModelString(modelString);

  // Step the world enough iterations to guarantee that the <insertions> is
  // recorded in the log file.
  world->Step(200);

  // Stop recording
  recorder->Stop();
  recorder->Fini();

  // Open the log file
  util::LogPlay *player = util::LogPlay::Instance();
  player->Open(filename);

  // The first step should be the initial world configuration
  std::string data;
  player->Step(data);
  EXPECT_NE(data.find("<world name='default'>"), std::string::npos);
  EXPECT_EQ(data.find("<insertions>"), std::string::npos);

  int count = 0;
  // The insertions should come after the first step
  while (player->Step(data))
  {
    size_t boxPos = data.find("<insertions><model name='box'>");
    while (boxPos != std::string::npos)
    {
      count++;
      boxPos = data.find("<insertions><model name='box'>", boxPos + 1);
    }
  }
  EXPECT_EQ(1, count) << "There were multiple insertions\n";

  // Cleanup the directory
  remove(filename.c_str());
  rmdir(tmpDir.c_str());
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
