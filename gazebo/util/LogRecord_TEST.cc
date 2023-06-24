/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/LogRecord.hh"
#include "test/util.hh"

class LogRecord_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test LogRecord constructor and a few accessors
TEST_F(LogRecord_TEST, Constructor)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  const char *homePath = common::getEnv(HOMEDIR);

  EXPECT_TRUE(homePath != NULL);

  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::filesystem::path logPath = paths->TmpPath() + "/gazebo";
  if (homePath)
    logPath = boost::filesystem::path(homePath);
  logPath /= "/.gazebo/log/";

  // Make sure the log path is correct
  EXPECT_EQ(recorder->BasePath(), logPath.string());

  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(recorder->Running());
  EXPECT_TRUE(recorder->FirstUpdate());

  // Init without a subdirectory
  EXPECT_FALSE(recorder->Init(""));
}

/////////////////////////////////////////////////
/// \brief Test LogRecord Start errors
TEST_F(LogRecord_TEST, StartErrors)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  // Start without an init
  {
    EXPECT_FALSE(recorder->Start("bz2"));
  }

  // Invalid encoding
  {
    EXPECT_TRUE(recorder->Init("test"));
    EXPECT_THROW(recorder->Start("garbage"), gazebo::common::Exception);
  }

  // Double start
  {
    EXPECT_TRUE(recorder->Start("bz2"));
    EXPECT_TRUE(recorder->Running());
    EXPECT_FALSE(recorder->Start("bz2"));
  }

  // Stop recording.
  recorder->Stop();

  // Make sure everything has reset.
  EXPECT_FALSE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_EQ(recorder->RunTime(), gazebo::common::Time());

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
/// \brief Test LogRecord Init and Start
TEST_F(LogRecord_TEST, Start_bzip2)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  EXPECT_TRUE(recorder->Init("test"));
  EXPECT_TRUE(recorder->Start("bz2"));

  // Make sure the right flags have been set
  EXPECT_FALSE(recorder->Paused());
  EXPECT_TRUE(recorder->Running());
  EXPECT_TRUE(recorder->FirstUpdate());

  // Make sure the right encoding is set
  EXPECT_EQ(recorder->Encoding(), std::string("bz2"));

  // Make sure the log directories exist
  EXPECT_TRUE(boost::filesystem::exists(recorder->BasePath()));
  EXPECT_TRUE(boost::filesystem::is_directory(recorder->BasePath()));

  // Run time should be zero since no update has been triggered.
  EXPECT_EQ(recorder->RunTime(), gazebo::common::Time());

  // Stop recording.
  recorder->Stop();

  // Make sure everything has reset.
  EXPECT_FALSE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_EQ(recorder->RunTime(), gazebo::common::Time());

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
/// \brief Test LogRecord Init and Start
TEST_F(LogRecord_TEST, Start_zlib)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  EXPECT_TRUE(recorder->Init("test"));
  EXPECT_TRUE(recorder->Start("zlib"));

  // Make sure the right flags have been set
  EXPECT_FALSE(recorder->Paused());
  EXPECT_TRUE(recorder->Running());
  EXPECT_TRUE(recorder->FirstUpdate());

  // Make sure the right encoding is set
  EXPECT_EQ(recorder->Encoding(), std::string("zlib"));

  // Make sure the log directories exist
  EXPECT_TRUE(boost::filesystem::exists(recorder->BasePath()));
  EXPECT_TRUE(boost::filesystem::is_directory(recorder->BasePath()));

  // Run time should be zero since no update has been triggered.
  EXPECT_EQ(recorder->RunTime(), gazebo::common::Time());

  // Stop recording.
  recorder->Stop();

  // Make sure everything has reset.
  EXPECT_FALSE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_EQ(recorder->RunTime(), gazebo::common::Time());

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
/// \brief Test LogRecord filter
TEST_F(LogRecord_TEST, Filter)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  // check default values
  EXPECT_DOUBLE_EQ(recorder->Period(), -1);
  EXPECT_EQ(recorder->Filter(), std::string());

  // filter by period
  recorder->SetPeriod(0.02);
  EXPECT_DOUBLE_EQ(recorder->Period(), 0.02);

  recorder->SetPeriod(0);
  EXPECT_DOUBLE_EQ(recorder->Period(), 0);


  // filter by regex string
  recorder->SetFilter("robot*");
  EXPECT_EQ(recorder->Filter(), "robot*");

  recorder->SetFilter("");
  EXPECT_EQ(recorder->Filter(), "");
}

/////////////////////////////////////////////////
/// \brief Test LogRecord record resources
TEST_F(LogRecord_TEST, RecordResources)
{
  gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();

  // check default values
  EXPECT_FALSE(recorder->RecordResources());

  recorder->SetRecordResources(true);
  EXPECT_TRUE(recorder->RecordResources());

  recorder->SetRecordResources(false);
  EXPECT_FALSE(recorder->RecordResources());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
