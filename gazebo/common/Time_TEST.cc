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

#include "gazebo/common/Time.hh"
#include "test/util.hh"

using namespace gazebo;

class TimeTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(TimeTest, Time)
{
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
TEST_F(TimeTest, ToString)
{
  common::Time time(0);

  // Several combinations
  EXPECT_EQ(time.FormattedString(), "00 00:00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::HOURS),
                                 "00:00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MINUTES),
                                 "00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::SECONDS),
                                 "00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MILLISECONDS),
                                 "000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::MILLISECONDS),
                                 "00 00:00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::SECONDS),
                                 "00 00:00:00");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::MINUTES),
                                 "00 00:00");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::HOURS),
                                 "00 00");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::DAYS),
                                 "00");

  // start > end: start pushes end
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MILLISECONDS,
                                 common::Time::FormatOption::MINUTES),
                                 "000");

  // 1 second
  time = common::Time(0, 1000000000);
  EXPECT_EQ(time.FormattedString(), "00 00:00:01.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MILLISECONDS,
                                 common::Time::FormatOption::MILLISECONDS),
                                 "1000");

  // 30.5 seconds
  time = common::Time(30, 500000000);
  EXPECT_EQ(time.FormattedString(), "00 00:00:30.500");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::SECONDS,
                                 common::Time::FormatOption::SECONDS),
                                 "30");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MILLISECONDS,
                                 common::Time::FormatOption::MILLISECONDS),
                                 "30500");

  // 1 min
  time = common::Time(60);
  EXPECT_EQ(time.FormattedString(), "00 00:01:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::SECONDS,
                                 common::Time::FormatOption::SECONDS),
                                 "60");

  // 2.5 hours
  time = common::Time(9000);
  EXPECT_EQ(time.FormattedString(), "00 02:30:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::HOURS,
                                 common::Time::FormatOption::MINUTES),
                                 "02:30");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::SECONDS,
                                 common::Time::FormatOption::SECONDS),
                                 "9000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MINUTES,
                                 common::Time::FormatOption::MINUTES),
                                 "150");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::HOURS,
                                 common::Time::FormatOption::HOURS),
                                 "02");

  // 3 days
  time = common::Time(259200);
  EXPECT_EQ(time.FormattedString(), "03 00:00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::DAYS,
                                 common::Time::FormatOption::DAYS),
                                 "03");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::HOURS),
                                 "72:00:00.000");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::HOURS,
                                 common::Time::FormatOption::HOURS),
                                 "72");
  EXPECT_EQ(time.FormattedString(common::Time::FormatOption::MINUTES,
                                 common::Time::FormatOption::MINUTES),
                                 "4320");

  // Large time
  time = common::Time(1234567890, 123456789);
  EXPECT_EQ(time.FormattedString(), "14288 23:31:30.123");
}

/////////////////////////////////////////////////
TEST_F(TimeTest, FromString)
{
  common::Time time;

  // Default start and end
  EXPECT_TRUE(time.SetFromFormattedString("00 00:00:01.000"));
  EXPECT_EQ(common::Time(1), time);

  // DAYS -> MILLISECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00 00:00:02.000",
      common::Time::FormatOption::DAYS,
      common::Time::FormatOption::MILLISECONDS));
  EXPECT_EQ(common::Time(2), time);

  // DAYS -> SECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00 00:00:03",
      common::Time::FormatOption::DAYS,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(3), time);

  // DAYS -> MINUTES
  EXPECT_TRUE(time.SetFromFormattedString("00 00:01",
      common::Time::FormatOption::DAYS,
      common::Time::FormatOption::MINUTES));
  EXPECT_EQ(common::Time(60), time);

  // DAYS -> HOURS
  EXPECT_TRUE(time.SetFromFormattedString("00 01",
      common::Time::FormatOption::DAYS,
      common::Time::FormatOption::HOURS));
  EXPECT_EQ(common::Time(3600), time);

  // DAYS -> DAYS
  EXPECT_TRUE(time.SetFromFormattedString("03",
      common::Time::FormatOption::DAYS,
      common::Time::FormatOption::DAYS));
  EXPECT_EQ(common::Time(259200), time);

  // HOURS -> MILLISECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00:00:02.000",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::MILLISECONDS));
  EXPECT_EQ(common::Time(2), time);

  // HOURS -> SECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00:00:03",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(3), time);

  // HOURS -> MINUTES
  EXPECT_TRUE(time.SetFromFormattedString("00:01",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::MINUTES));
  EXPECT_EQ(common::Time(60), time);

  // HOURS -> HOURS
  EXPECT_TRUE(time.SetFromFormattedString("01",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::HOURS));
  EXPECT_EQ(common::Time(3600), time);

  // MINUTES -> MILLISECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00:02.000",
      common::Time::FormatOption::MINUTES,
      common::Time::FormatOption::MILLISECONDS));
  EXPECT_EQ(common::Time(2), time);

  // MINUTES -> SECONDS
  EXPECT_TRUE(time.SetFromFormattedString("00:03",
      common::Time::FormatOption::MINUTES,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(3), time);

  // MINUTES -> MINUTES
  EXPECT_TRUE(time.SetFromFormattedString("01",
      common::Time::FormatOption::MINUTES,
      common::Time::FormatOption::MINUTES));
  EXPECT_EQ(common::Time(60), time);

  // SECONDS -> MILLISECONDS
  EXPECT_TRUE(time.SetFromFormattedString("02.000",
      common::Time::FormatOption::SECONDS,
      common::Time::FormatOption::MILLISECONDS));
  EXPECT_EQ(common::Time(2), time);

  // SECONDS -> SECONDS
  EXPECT_TRUE(time.SetFromFormattedString("03",
      common::Time::FormatOption::SECONDS,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(3), time);

  // MILLISECONDS -> MILLISECONDS
  EXPECT_TRUE(time.SetFromFormattedString("000",
      common::Time::FormatOption::MILLISECONDS,
      common::Time::FormatOption::MILLISECONDS));
  EXPECT_EQ(common::Time(0), time);

  // Give wrong start / end and check it wasn't set
  EXPECT_FALSE(time.SetFromFormattedString("00 00:00:03.000",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(0), time);

  // Give bad string and check it wasn't set
  EXPECT_FALSE(time.SetFromFormattedString("xyz",
      common::Time::FormatOption::HOURS,
      common::Time::FormatOption::SECONDS));
  EXPECT_EQ(common::Time(0), time);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
