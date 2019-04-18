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
#include <gtest/gtest.h>

#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/test/ServerFixture.hh"

#include "ignition/math/Pose3.hh"


using namespace gazebo;

/// \brief A test fixture class.
class IntrospectionManagerTest : public ::testing::Test
{
  /// \brief Constructor
  public: IntrospectionManagerTest()
  {
    this->manager = util::IntrospectionManager::Instance();
  }

  /// \brief Initialize the test
  public: void SetUp()
  {
  }

  public: void TearDown()
  {
    // Unregister multiple items.
    this->manager->Clear();
    EXPECT_TRUE(this->manager->Items().empty());
  }

  /// \brief Pointer to the introspection manager.
  protected: util::IntrospectionManager *manager;
};


TEST_F(IntrospectionManagerTest, IntrospectionManagerStressTest)
{
  // Each model registers 5 items (pos, linvel, angvel, linaccel, angaccel)
  // This would be the equivalent of adding 2000 models.
  for (size_t ii = 0; ii < 10000; ii++)
  {
    // A callback for updating items.
    // This arbitrarily captures something large enough to not
    // use SBO for std::function.
    auto func = [this,
                 a = std::string("asdfasdfasdfasdf"),
                 b = std::string("asdfasdfasdfasdf"),
                 c = std::string("asdfasdfasdfasdf"),
                 d = std::string("asdfasdfasdfasdf")]()
    {
      return a + b + c + d;
    };

    std::stringstream ss;
    ss << "item" << ii;
    EXPECT_TRUE(this->manager->Register<std::string>(ss.str(), func));
  }

  std::vector<double> times;
  for (size_t ii = 0; ii < 1000; ++ii)
  {
    common::Time startTime = common::Time::GetWallTime();
    this->manager->Update();
    common::Time endTime = common::Time::GetWallTime();
    times.push_back((endTime - startTime).Double());
  }

  auto n = times.size();
  std::sort(times.begin(), times.end());
  auto sum = std::accumulate(times.begin(), times.end(), 0.0);

  std::cerr << "Samples: " << n << std::endl;
  std::cerr << "Max: " << times.back() << std::endl;
  std::cerr << "Min: " << times.front() << std::endl;
  // Not exactly median, but really close.
  std::cerr << "Median: " << times[n/2] << std::endl;
  std::cerr << "Mean: " << sum / static_cast<double>(n) << std::endl;
}
