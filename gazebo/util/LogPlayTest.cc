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

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/LogPlay.hh"
#include "test/util.hh"

class LogPlay_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test LogPlay Open.
TEST_F(LogPlay_TEST, Open)
{
  gazebo::util::LogPlay *player = gazebo::util::LogPlay::Instance();
  EXPECT_FALSE(player->IsOpen());

  EXPECT_TRUE(recorder->Init("test"));
  EXPECT_TRUE(recorder->Start("zlib"));

  // Make sure the right flags have been set
  EXPECT_FALSE(recorder->GetPaused());
  EXPECT_TRUE(recorder->GetRunning());
  EXPECT_TRUE(recorder->GetFirstUpdate());

  // Make sure the right encoding is set
  EXPECT_EQ(recorder->GetEncoding(), std::string("zlib"));

  // Make sure the log directories exist
  EXPECT_TRUE(boost::filesystem::exists(recorder->GetBasePath()));
  EXPECT_TRUE(boost::filesystem::is_directory(recorder->GetBasePath()));

  // Run time should be zero since no update has been triggered.
  EXPECT_EQ(recorder->GetRunTime(), gazebo::common::Time());

  // Stop recording.
  recorder->Stop();

  // Make sure everything has reset.
  EXPECT_FALSE(recorder->GetRunning());
  EXPECT_FALSE(recorder->GetPaused());
  EXPECT_EQ(recorder->GetRunTime(), gazebo::common::Time());

  // Logger may still be writing so make sure we exit cleanly
  int i = 0;
  while (!recorder->IsReadyToStart())
  {
    gazebo::common::Time::MSleep(100);
    if ((++i % 50) == 0)
      gzdbg << "Waiting for recorder->IsReadyToStart()" << std::endl;
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
