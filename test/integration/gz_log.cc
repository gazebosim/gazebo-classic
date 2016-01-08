/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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

#include "gazebo/util/LogRecord.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class GzLog : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test log recording from gzserver
TEST_F(GzLog, Record)
{
  util::LogRecord *recorder = util::LogRecord::Instance();
  recorder->Init("test");
  Load("worlds/single_revolute_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  world->SetPaused(true);

  ASSERT_TRUE(recorder != NULL);

  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(recorder->Running());

  // Start log recording
  custom_exec("gz log -w default -d 1");
  world->Step(100);

  std::string filename = recorder->Filename();

  EXPECT_TRUE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(filename.empty());
  EXPECT_GT(recorder->FileSize(), 0u);

  // Stop log recording
  custom_exec("gz log -w default -d 0");

  EXPECT_FALSE(recorder->Running());

  std::string cmd = "gz log -i -f " + filename;
  std::string info = custom_exec(cmd);
  std::string logVersionStr = info.substr(
      info.find("Log Version:    ")+16, info.find("\n")-16);

  EXPECT_EQ(logVersionStr, GZ_LOG_VERSION);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
