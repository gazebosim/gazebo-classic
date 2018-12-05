/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

#include <gtest/gtest.h>

#include "gazebo/common/FuelModelDatabase.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
/// \brief In this test we verify that there aren't any exceptions when the
/// server is incorrect.
TEST(FuelModelDatabaseTest, FuelDown)
{
  // We use the real FuelModelDatabase class here.
  auto fuelDB = FuelModelDatabase::Instance();
  std::mutex mutex;
  std::condition_variable cv;
  bool cbExecuted = false;

  std::function <void(
      const std::vector<ignition::fuel_tools::ModelIdentifier> &)> cb =
      [&cbExecuted, &cv](
      const std::vector<ignition::fuel_tools::ModelIdentifier> &_models)
        {
          cbExecuted = true;

          // Verify the content received. The server shouldn't exist, so we
          // shouldn't receive any models.
          EXPECT_EQ(0u, _models.size());

          cv.notify_all();
        };

  ignition::fuel_tools::ServerConfig srv;
  srv.URL("___bad_server___");
  fuelDB->Models(srv, cb);

  // We now wait for a while unless the callback is executed and wake us up.
  std::unique_lock<std::mutex> lk(mutex);
  auto now = std::chrono::system_clock::now();
  EXPECT_TRUE(cv.wait_until(lk, now + std::chrono::milliseconds(200),
    [&cbExecuted]
    {
      return cbExecuted;
    }));
}

/////////////////////////////////////////////////
/// \brief In this test we verify that Servers() returns at least one Fuel
/// server.
TEST(FuelModelDatabaseTest, Servers)
{
  auto fuelDB = FuelModelDatabase::Instance();
  EXPECT_GT(fuelDB->Servers().size(), 0u);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
