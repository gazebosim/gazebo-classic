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
#endif
#include <gazebo/util/LogRecord.hh>
#include <gazebo/util/LogPlay.hh>
#include <gazebo/test/ServerFixture.hh>

using namespace gazebo;

class Issue2428_Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// This test checks that the following steps work:
//
// 1. run Gazebo
// 2. Insert some models, at least one of them should move
// 3. Record a log
// 4. Stop record
// 5. The log file should have the inserted model inside the
// <world>...</world> block.
TEST_F(Issue2428_Test, InsertionBeforeRecording)
{
#ifndef _WIN32
  // Create a temporary directory
  char dirTemplate[] ="/tmp/gazeboXXXXXX";
  std::string tmpDir = mkdtemp(dirTemplate);
#else
  boost::filesystem::path tmppath = boost::filesystem::temp_directory_path();
  std::string tmpDir = tmppath.string();
#endif

  util::LogRecord *recorder = util::LogRecord::Instance();
  recorder->Init("test");

  // Playback the state log
  this->Load("worlds/empty.world", true);

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // take some steps before inserting model to avoid issue #2297
  world->Step(100);

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

  // Insert a model before recording starts
  world->InsertModelString(modelString);
  world->Step(100);

  recorder->Start("txt", tmpDir);

  // Step the world enough iterations to guarantee that the <insertions> is
  // recorded in the log file.
  world->Step(200);

  std::string filename = recorder->Filename();

  // Stop recording
  recorder->Stop();
  recorder->Fini();

  // Open the log file
  util::LogPlay *player = util::LogPlay::Instance();
  player->Open(filename);

  std::string data;
  player->Step(data);
  size_t worldStart = data.find("<world name='default'>");
  size_t boxPos = data.find("<model name='box'>");
  size_t worldEnd = data.find("</world>");

  // The box model should exist in the initial world.
  EXPECT_LT(worldStart, boxPos);
  EXPECT_GT(worldEnd, boxPos);
  EXPECT_NE(boxPos, std::string::npos);

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
