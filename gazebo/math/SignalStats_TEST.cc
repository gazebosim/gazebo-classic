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

#include <gtest/gtest.h>

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/SignalStats.hh"
#include "test/util.hh"

using namespace gazebo;

class SignalStatsTest : public gazebo::testing::AutoLogFixture { };

TEST_F(SignalStatsTest, SignalMean)
{
  {
    // Constructor
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
    EXPECT_EQ(mean.GetCount(), 0u);
    EXPECT_EQ(mean.GetShortName(), std::string("Mean"));

    // Reset
    mean.Reset();
    EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
    EXPECT_EQ(mean.GetCount(), 0u);
  }

  {
    // Constant values, mean should match
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
    EXPECT_EQ(mean.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        mean.InsertData(value);
        EXPECT_NEAR(mean.Get(), value, 1e-10);
        EXPECT_EQ(mean.GetCount(), i);
      }

      // Reset
      mean.Reset();
      EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
      EXPECT_EQ(mean.GetCount(), 0u);
    }
  }

  {
    // Values with alternating sign, increasing magnitude
    // Should be zero every other time
    math::SignalMean mean;
    EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
    EXPECT_EQ(mean.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        mean.InsertData(value * i);
        mean.InsertData(-value * i);
        EXPECT_NEAR(mean.Get(), 0.0, 1e-10);
        EXPECT_EQ(mean.GetCount(), i*2);
      }

      // Reset
      mean.Reset();
      EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
      EXPECT_EQ(mean.GetCount(), 0u);
    }
  }
}

TEST_F(SignalStatsTest, SignalRootMeanSquare)
{
  {
    // Constructor
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
    EXPECT_EQ(rms.GetCount(), 0u);
    EXPECT_EQ(rms.GetShortName(), std::string("Rms"));

    // Reset
    rms.Reset();
    EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
    EXPECT_EQ(rms.GetCount(), 0u);
  }

  {
    // Constant values, rms should match
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
    EXPECT_EQ(rms.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        rms.InsertData(value);
        EXPECT_NEAR(rms.Get(), value, 1e-10);
        EXPECT_EQ(rms.GetCount(), i);
      }

      // Reset
      rms.Reset();
      EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
      EXPECT_EQ(rms.GetCount(), 0u);
    }
  }

  {
    // Values with alternating sign, same magnitude
    // rms should match absolute value every time
    math::SignalRootMeanSquare rms;
    EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
    EXPECT_EQ(rms.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        rms.InsertData(value);
        EXPECT_NEAR(rms.Get(), value, 1e-10);
        EXPECT_EQ(rms.GetCount(), i*2-1);

        rms.InsertData(-value);
        EXPECT_NEAR(rms.Get(), value, 1e-10);
        EXPECT_EQ(rms.GetCount(), i*2);
      }

      // Reset
      rms.Reset();
      EXPECT_DOUBLE_EQ(rms.Get(), 0.0);
      EXPECT_EQ(rms.GetCount(), 0u);
    }
  }
}

TEST_F(SignalStatsTest, SignalMaxAbsoluteValue)
{
  {
    // Constructor
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Get(), 0.0);
    EXPECT_EQ(max.GetCount(), 0u);
    EXPECT_EQ(max.GetShortName(), std::string("MaxAbs"));

    // Reset
    max.Reset();
    EXPECT_DOUBLE_EQ(max.Get(), 0.0);
    EXPECT_EQ(max.GetCount(), 0u);
  }

  {
    // Constant values, max should match
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Get(), 0.0);
    EXPECT_EQ(max.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        max.InsertData(value);
        EXPECT_NEAR(max.Get(), value, 1e-10);
        EXPECT_EQ(max.GetCount(), i);
      }

      // Reset
      max.Reset();
      EXPECT_DOUBLE_EQ(max.Get(), 0.0);
      EXPECT_EQ(max.GetCount(), 0u);
    }
  }

  {
    // Values with alternating sign, increasing magnitude
    // max should match absolute value every time
    math::SignalMaxAbsoluteValue max;
    EXPECT_DOUBLE_EQ(max.Get(), 0.0);
    EXPECT_EQ(max.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        max.InsertData(value * i);
        EXPECT_NEAR(max.Get(), value * i, 1e-10);
        EXPECT_EQ(max.GetCount(), i*2-1);

        max.InsertData(-value * i);
        EXPECT_NEAR(max.Get(), value * i, 1e-10);
        EXPECT_EQ(max.GetCount(), i*2);
      }

      // Reset
      max.Reset();
      EXPECT_DOUBLE_EQ(max.Get(), 0.0);
      EXPECT_EQ(max.GetCount(), 0u);
    }
  }
}

TEST_F(SignalStatsTest, SignalStats)
{
  {
    // Constructor
    math::SignalStats stats;
    EXPECT_TRUE(stats.GetMap().empty());
    EXPECT_EQ(stats.GetCount(), 0u);

    // Reset
    stats.Reset();
    EXPECT_TRUE(stats.GetMap().empty());
    EXPECT_EQ(stats.GetCount(), 0u);
  }

  {
    // InsertStatistic
    math::SignalStats stats;
    EXPECT_TRUE(stats.GetMap().empty());

    EXPECT_TRUE(stats.InsertStatistic("MaxAbs"));
    EXPECT_FALSE(stats.InsertStatistic("MaxAbs"));
    EXPECT_FALSE(stats.GetMap().empty());

    EXPECT_TRUE(stats.InsertStatistic("Mean"));
    EXPECT_FALSE(stats.InsertStatistic("Mean"));
    EXPECT_FALSE(stats.GetMap().empty());

    EXPECT_TRUE(stats.InsertStatistic("Rms"));
    EXPECT_FALSE(stats.InsertStatistic("Rms"));
    EXPECT_FALSE(stats.GetMap().empty());

    EXPECT_FALSE(stats.InsertStatistic("FakeStatistic"));

    // GetMap with no data
    std::map<std::string, double> map = stats.GetMap();
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), 3u);
    EXPECT_EQ(map.count("MaxAbs"), 1u);
    EXPECT_EQ(map.count("Mean"), 1u);
    EXPECT_EQ(map.count("Rms"), 1u);
    EXPECT_EQ(map.count("FakeStatistic"), 0u);
  }

  {
    // InsertStatistics
    math::SignalStats stats;
    EXPECT_TRUE(stats.GetMap().empty());

    EXPECT_TRUE(stats.InsertStatistics("MaxAbs,Rms"));
    EXPECT_FALSE(stats.InsertStatistics("MaxAbs,Rms"));
    EXPECT_FALSE(stats.InsertStatistics("MaxAbs"));
    EXPECT_FALSE(stats.InsertStatistics("Rms"));
    EXPECT_FALSE(stats.GetMap().empty());

    EXPECT_FALSE(stats.InsertStatistics("Mean,FakeStatistic"));
    EXPECT_FALSE(stats.GetMap().empty());

    EXPECT_FALSE(stats.InsertStatistics("FakeStatistic"));

    // GetMap with no data
    std::map<std::string, double> map = stats.GetMap();
    EXPECT_FALSE(map.empty());
    EXPECT_EQ(map.size(), 3u);
    EXPECT_EQ(map.count("MaxAbs"), 1u);
    EXPECT_EQ(map.count("Mean"), 1u);
    EXPECT_EQ(map.count("Rms"), 1u);
    EXPECT_EQ(map.count("FakeStatistic"), 0u);
  }

  {
    // Add some statistics
    math::SignalStats stats;
    EXPECT_TRUE(stats.InsertStatistics("MaxAbs,Mean,Rms"));
    EXPECT_EQ(stats.GetMap().size(), 3u);

    // No data yet
    EXPECT_EQ(stats.GetCount(), 0u);

    // Insert data with alternating signs
    const double value = 3.14159;
    stats.InsertData(value);
    stats.InsertData(-value);
    EXPECT_EQ(stats.GetCount(), 2u);

    {
      std::map<std::string, double> map = stats.GetMap();
      EXPECT_NEAR(map["MaxAbs"], value, 1e-10);
      EXPECT_NEAR(map["Rms"], value, 1e-10);
      EXPECT_NEAR(map["Mean"], 0.0, 1e-10);
    }

    stats.Reset();
    EXPECT_EQ(stats.GetMap().size(), 3u);
    EXPECT_EQ(stats.GetCount(), 0u);
    {
      std::map<std::string, double> map = stats.GetMap();
      EXPECT_DOUBLE_EQ(map["MaxAbs"], 0.0);
      EXPECT_DOUBLE_EQ(map["Rms"], 0.0);
      EXPECT_DOUBLE_EQ(map["Mean"], 0.0);
    }
  }
}

TEST_F(SignalStatsTest, Vector3Stats)
{
  {
    // Constructor
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.x.GetMap().empty());
    EXPECT_TRUE(v3stats.y.GetMap().empty());
    EXPECT_TRUE(v3stats.z.GetMap().empty());
    EXPECT_TRUE(v3stats.mag.GetMap().empty());
    EXPECT_EQ(v3stats.x.GetCount(), 0u);
    EXPECT_EQ(v3stats.y.GetCount(), 0u);
    EXPECT_EQ(v3stats.z.GetCount(), 0u);
    EXPECT_EQ(v3stats.mag.GetCount(), 0u);

    // Reset
    v3stats.Reset();
    EXPECT_TRUE(v3stats.x.GetMap().empty());
    EXPECT_TRUE(v3stats.y.GetMap().empty());
    EXPECT_TRUE(v3stats.z.GetMap().empty());
    EXPECT_TRUE(v3stats.mag.GetMap().empty());
    EXPECT_EQ(v3stats.x.GetCount(), 0u);
    EXPECT_EQ(v3stats.y.GetCount(), 0u);
    EXPECT_EQ(v3stats.z.GetCount(), 0u);
    EXPECT_EQ(v3stats.mag.GetCount(), 0u);
  }

  {
    // InsertStatistics
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.x.GetMap().empty());
    EXPECT_TRUE(v3stats.y.GetMap().empty());
    EXPECT_TRUE(v3stats.z.GetMap().empty());
    EXPECT_TRUE(v3stats.mag.GetMap().empty());

    EXPECT_TRUE(v3stats.InsertStatistics("MaxAbs"));
    EXPECT_FALSE(v3stats.InsertStatistics("MaxAbs"));
    EXPECT_FALSE(v3stats.x.GetMap().empty());
    EXPECT_FALSE(v3stats.y.GetMap().empty());
    EXPECT_FALSE(v3stats.z.GetMap().empty());
    EXPECT_FALSE(v3stats.mag.GetMap().empty());

    // GetMap with no data
    {
      std::map<std::string, double> map = v3stats.x.GetMap();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.y.GetMap();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.z.GetMap();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.mag.GetMap();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }

    // Insert some data
    EXPECT_EQ(v3stats.x.GetCount(), 0u);
    EXPECT_EQ(v3stats.y.GetCount(), 0u);
    EXPECT_EQ(v3stats.z.GetCount(), 0u);
    EXPECT_EQ(v3stats.mag.GetCount(), 0u);

    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitY);

    EXPECT_EQ(v3stats.x.GetCount(), 3u);
    EXPECT_EQ(v3stats.y.GetCount(), 3u);
    EXPECT_EQ(v3stats.z.GetCount(), 3u);
    EXPECT_EQ(v3stats.mag.GetCount(), 3u);

    EXPECT_NEAR(v3stats.x.GetMap()["MaxAbs"], 1.0, 1e-10);
    EXPECT_NEAR(v3stats.y.GetMap()["MaxAbs"], 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(v3stats.z.GetMap()["MaxAbs"], 0.0);
    EXPECT_NEAR(v3stats.mag.GetMap()["MaxAbs"], 1.0, 1e-10);
  }
}

