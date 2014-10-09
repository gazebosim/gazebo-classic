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

#include "gazebo/math/Vector3Stats.hh"
#include "test/util.hh"

using namespace gazebo;

class Vector3StatsTest : public gazebo::testing::AutoLogFixture { };

TEST_F(Vector3StatsTest, Vector3Stats)
{
  {
    // Constructor
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);

    // Reset
    v3stats.Reset();
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);
  }

  {
    // InsertStatistics
    math::Vector3Stats v3stats;
    EXPECT_TRUE(v3stats.X().Map().empty());
    EXPECT_TRUE(v3stats.Y().Map().empty());
    EXPECT_TRUE(v3stats.Z().Map().empty());
    EXPECT_TRUE(v3stats.Mag().Map().empty());

    EXPECT_TRUE(v3stats.InsertStatistics("MaxAbs"));
    EXPECT_FALSE(v3stats.InsertStatistics("MaxAbs"));
    EXPECT_FALSE(v3stats.X().Map().empty());
    EXPECT_FALSE(v3stats.Y().Map().empty());
    EXPECT_FALSE(v3stats.Z().Map().empty());
    EXPECT_FALSE(v3stats.Mag().Map().empty());

    // Map with no data
    {
      std::map<std::string, double> map = v3stats.X().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Y().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Z().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }
    {
      std::map<std::string, double> map = v3stats.Mag().Map();
      EXPECT_EQ(map.size(), 1u);
      EXPECT_EQ(map.count("MaxAbs"), 1u);
    }

    // Insert some data
    EXPECT_EQ(v3stats.X().Count(), 0u);
    EXPECT_EQ(v3stats.Y().Count(), 0u);
    EXPECT_EQ(v3stats.Z().Count(), 0u);
    EXPECT_EQ(v3stats.Mag().Count(), 0u);

    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitX);
    v3stats.InsertData(math::Vector3::UnitY);

    EXPECT_EQ(v3stats.X().Count(), 3u);
    EXPECT_EQ(v3stats.Y().Count(), 3u);
    EXPECT_EQ(v3stats.Z().Count(), 3u);
    EXPECT_EQ(v3stats.Mag().Count(), 3u);

    EXPECT_NEAR(v3stats.X().Map()["MaxAbs"], 1.0, 1e-10);
    EXPECT_NEAR(v3stats.Y().Map()["MaxAbs"], 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(v3stats.Z().Map()["MaxAbs"], 0.0);
    EXPECT_NEAR(v3stats.Mag().Map()["MaxAbs"], 1.0, 1e-10);
  }
}

