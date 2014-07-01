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
        mean.Insert(value);
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
    // Values with alternating sign, same magnitude
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
        mean.Insert(value);
        mean.Insert(-value);
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
        rms.Insert(value);
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
    math::SignalRootMeanSquare mean;
    EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
    EXPECT_EQ(mean.GetCount(), 0u);

    const double value = 3.14159;

    // Loop two times to verify Reset
    for (int j = 0; j < 2; ++j)
    {
      for (unsigned int i = 1; i <= 10; ++i)
      {
        mean.Insert(value);
        EXPECT_NEAR(mean.Get(), value, 1e-10);
        EXPECT_EQ(mean.GetCount(), i*2-1);

        mean.Insert(-value);
        EXPECT_NEAR(mean.Get(), value, 1e-10);
        EXPECT_EQ(mean.GetCount(), i*2);
      }

      // Reset
      mean.Reset();
      EXPECT_DOUBLE_EQ(mean.Get(), 0.0);
      EXPECT_EQ(mean.GetCount(), 0u);
    }
  }
}
