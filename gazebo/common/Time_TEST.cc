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

#include <sys/time.h>
#include <gtest/gtest.h>

#include "gazebo/common/Timer.hh"
#include "gazebo/common/Time.hh"
#include "test/util.hh"

using namespace gazebo;

class TimeTest : public gazebo::testing::AutoLogFixture { };

TEST_F(TimeTest, Time)
{
  common::Timer timer;
  timer.Start();
  common::Time::MSleep(100);
  EXPECT_TRUE(timer.GetElapsed() > common::Time(0, 100000000));

  struct timeval tv;
  gettimeofday(&tv, NULL);
  common::Time time(tv);
  EXPECT_EQ(time.sec, tv.tv_sec);
  EXPECT_EQ(time.nsec, tv.tv_usec * 1000);

  time.SetToWallTime();
  EXPECT_TRUE(common::Time::GetWallTime() - time < common::Time(0, 1000000));

  time = common::Time(1, 1000)
       + common::Time(1.5)
       + common::Time(0, 1e9);
  EXPECT_EQ(time, common::Time(3, 5e8 + 1000));

  time.Set(1, 1000);
  time += common::Time(1.5);
  time += common::Time(0, 1e9);
  EXPECT_EQ(time, common::Time(3, 5e8 + 1000));

  time.Set(1, 1000);
  time -= common::Time(1, 1000);
  EXPECT_TRUE(time == common::Time(0, 0));

  time.Set(1, 1000);
  time *= common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(2, 2002));

  time.Set(2, 4000);
  time /= common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(1, 1999));
  EXPECT_FALSE(time != common::Time(1, 1999));

  time += common::Time(0, 1);
  tv.tv_sec = 1;
  tv.tv_usec = 2;
  EXPECT_TRUE(time == tv);
  EXPECT_FALSE(time != tv);

  tv.tv_sec = 2;
  EXPECT_TRUE(time < tv);

  tv.tv_sec = 0;
  EXPECT_TRUE(time > tv);
  EXPECT_TRUE(time >= tv);


  EXPECT_TRUE(time == 1.0 + 2000*1e-9);
  EXPECT_FALSE(time != 1.0 + 2000*1e-9);
  EXPECT_TRUE(time < 2.0);
  EXPECT_TRUE(time > 0.1);
  EXPECT_TRUE(time >= 0.1);


  tv.tv_sec = 2;
  tv.tv_usec = 1000000;
  time = common::Time(1, 1000) + tv;
  EXPECT_TRUE(time == common::Time(4.0, 1000));

  time.Set(1, 1000);
  time += tv;
  EXPECT_TRUE(time == common::Time(4, 1000));

  time = common::Time(1, 1000) - tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  time.Set(1, 1000);
  time -= tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  tv.tv_sec = 2;
  tv.tv_usec = 1000;
  time = common::Time(1, 1000) * tv;
  EXPECT_TRUE(time == common::Time(2, 1002001));

  time.Set(1, 1000);
  time *= tv;
  EXPECT_TRUE(time == common::Time(2, 1002001));

  time.Set(1, 1000);
  time = common::Time(1, 1000) * common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(2, 2002));


  time = common::Time(1, 2000000) / tv;
  EXPECT_TRUE(time == common::Time(0, 500749625));

  time.Set(1, 2000000);
  time /= tv;
  EXPECT_TRUE(time == common::Time(0, 500749625));

  time.Set(1, 1000);
  time = common::Time(1, 1000) / common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(0, 500000499));

  double sec = 1.0 + 1e-9;
  double msec = sec * 1e3;
  double usec = sec * 1e6;
  double nsec = sec * 1e9;
  EXPECT_DOUBLE_EQ(nsec, common::Time::SecToNano(sec));
  EXPECT_DOUBLE_EQ(nsec, common::Time::MilToNano(msec));
  EXPECT_DOUBLE_EQ(nsec, common::Time::MicToNano(usec));
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
