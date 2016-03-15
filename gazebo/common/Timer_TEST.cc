/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Timer.hh"
#include "test/util.hh"

using namespace gazebo;

class TimerTest : public gazebo::testing::AutoLogFixture
{
};

/////////////////////////////////////////////////
TEST_F(TimerTest, Timer)
{
  common::Timer timer;
  // Expect the timer to be paused on initialization and at 0 time
  EXPECT_EQ(timer.GetElapsed().sec, 0);
  EXPECT_EQ(timer.GetElapsed().nsec, 0);
  EXPECT_FALSE(timer.GetRunning());

  // Run for 200 ms then check elapsed time
  // Save expectations until after timer is stopped
  timer.Start();
  common::Time::MSleep(200);
  EXPECT_TRUE(timer.GetRunning());
  common::Time elapsed200ms = timer.GetElapsed();

  // Check that the time after stopping and sleeping matches the time right
  // before pausing
  timer.Stop();
  common::Time::MSleep(100);

  // Do expectations now that timer is stopped.
  EXPECT_GE(elapsed200ms.nsec, 199950000);
#ifdef __APPLE__
  EXPECT_LE(elapsed200ms.nsec, 206000000);
#else
  EXPECT_LE(elapsed200ms.nsec, 200200000);
#endif
  EXPECT_EQ(elapsed200ms.sec, 0);

#ifdef __APPLE__
  EXPECT_GE(elapsed200ms.nsec, timer.GetElapsed().nsec - 300000);
#else
  EXPECT_GE(elapsed200ms.nsec, timer.GetElapsed().nsec - 100000);
#endif
  EXPECT_LE(elapsed200ms.nsec, timer.GetElapsed().nsec + 100000);
  EXPECT_EQ(elapsed200ms.sec, timer.GetElapsed().sec);
  EXPECT_FALSE(timer.GetRunning());

  // Expect that we start from where we left off
  timer.Start();
  common::Time::MSleep(100);
  common::Time elapsed300ms = timer.GetElapsed();
  EXPECT_GE(elapsed300ms.nsec, 300000000);
#ifdef __APPLE__
  EXPECT_LE(elapsed300ms.nsec, 311000000);
#else
  EXPECT_LE(elapsed300ms.nsec, 301000000);
#endif
  EXPECT_EQ(elapsed300ms.sec, 0);

  // Expect reset to reset the current time and stop the timer
  timer.Reset();
  EXPECT_EQ(timer.GetElapsed().sec, 0);
  EXPECT_EQ(timer.GetElapsed().nsec, 0);
  EXPECT_FALSE(timer.GetRunning());
  common::Time::MSleep(1);
  EXPECT_EQ(timer.GetElapsed().sec, 0);
  EXPECT_EQ(timer.GetElapsed().nsec, 0);
  EXPECT_FALSE(timer.GetRunning());
}

/////////////////////////////////////////////////
TEST_F(TimerTest, CountdownTimer)
{
  // Count down from 1 second
  common::Time maxTime(1);
  common::Timer timer(maxTime, true);
  EXPECT_EQ(timer.GetElapsed().sec, 1);
  EXPECT_EQ(timer.GetElapsed().nsec, 0);
  EXPECT_FALSE(timer.GetRunning());

  // Run for 200 ms then check elapsed time
  // Save expectations until after timer is stopped
  timer.Start();
  common::Time::MSleep(200);
  EXPECT_TRUE(timer.GetRunning());
  common::Time elapsed200ms = timer.GetElapsed();

  // Check that the time after stopping and sleeping matches the time right
  // before pausing
  timer.Stop();
  common::Time::MSleep(100);

  // Do expectations now that timer is stopped.
#ifdef __APPLE__
  EXPECT_GE(elapsed200ms.nsec, 793000000);
#else
  EXPECT_GE(elapsed200ms.nsec, 799800000);
#endif
  EXPECT_LE(elapsed200ms.nsec, 800100000);
  EXPECT_EQ(elapsed200ms.sec, 0);

  EXPECT_GE(elapsed200ms.nsec, timer.GetElapsed().nsec - 100000);
  EXPECT_LE(elapsed200ms.nsec, timer.GetElapsed().nsec + 100000);
  EXPECT_EQ(elapsed200ms.sec, timer.GetElapsed().sec);
  EXPECT_FALSE(timer.GetRunning());

  // Expect that we start from where we left off
  timer.Start();
  common::Time::MSleep(100);
#ifdef __APPLE__
  EXPECT_GE(timer.GetElapsed().nsec, 688000000);
#else
  EXPECT_GE(timer.GetElapsed().nsec, 699000000);
#endif
  EXPECT_LE(timer.GetElapsed().nsec, 701000000);
  EXPECT_EQ(timer.GetElapsed().sec, 0);

  // Expect reset to reset the current time and stop the timer
  timer.Reset();
  EXPECT_EQ(timer.GetElapsed().sec, 1);
  EXPECT_EQ(timer.GetElapsed().nsec, 0);
  EXPECT_FALSE(timer.GetRunning());
}
