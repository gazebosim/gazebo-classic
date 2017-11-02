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
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "gazebo/common/FuelModelDatabase.hh"

using namespace gazebo;
using namespace common;

/// \brief A class used to mock the FuelModelDatabase.
/// This will avoid a real request over the network.
class FuelModelDatabaseMock : public FuelModelDatabase
{
  /// \brief Constructor.
  public: FuelModelDatabaseMock(const std::string &_server)
    : FuelModelDatabase(_server)
  {
  }

  /// \brief Override the real asynchronous Models() call.
  public: void Models(
      std::function<void (const std::map<std::string, std::string> &)> &_func)
  {
    std::thread t([this, _func]
    {
      // Run the callback passing the list of models.
      _func(this->Models());
    });
    t.detach();
  }

  /// \brief Override the real synchronous Models() call.
  public: std::map<std::string, std::string> Models()
  {
    std::map<std::string, std::string> models;
    models["my_server/owner/model_1"] = "model_1";
    models["my_server/owner/model_2"] = "model_2";

    return models;
  }
};

/////////////////////////////////////////////////
/// \brief In this test we verify that there aren't any exceptions when the
/// server is incorrect.
TEST(FuelModelDatabaseTest, FuelDown)
{
  // We use the real FuelModelDatabase class here.
  FuelModelDatabase fuelDB("___my_wrong_server___");
  std::mutex mutex;
  std::condition_variable cv;
  bool cbExecuted = false;

  std::function<void(const std::map<std::string, std::string> &)> cb =
        [&cbExecuted, &cv](const std::map<std::string, std::string> &_models)
        {
          cbExecuted = true;

          // Verify the content received. The server shouldn't exist, so we
          // shouldn't receive any models.
          EXPECT_EQ(0u, _models.size());

          cv.notify_all();
        };

  fuelDB.Models(cb);

  // We now wait for a while unless the callback is executed and wake us up.
  std::unique_lock<std::mutex> lk(mutex);
  auto now = std::chrono::system_clock::now();
  EXPECT_TRUE(cv.wait_until(lk, now + std::chrono::milliseconds(100),
    [&cbExecuted]
    {
      return cbExecuted;
    }));
}

/////////////////////////////////////////////////
/// \brief In this test we verify the asynchronous version of
/// FuelModelDatabase::Models().
TEST(FuelModelDatabaseTest, ModelsAsync)
{
  // We don't use the real FuelModelDatabase class. We want to prevent
  // generating real request against production Fuel servers.
  FuelModelDatabaseMock fuelDB("my_server");
  std::mutex mutex;
  std::condition_variable cv;
  bool cbExecuted = false;

  std::function<void(const std::map<std::string, std::string> &)> cb =
        [&cbExecuted, &cv](const std::map<std::string, std::string> &_models)
        {
          cbExecuted = true;

          // Verify the content received.
          EXPECT_EQ(2u, _models.size());
          ASSERT_NE(_models.end(), _models.find("my_server/owner/model_1"));
          EXPECT_EQ("model_1", _models.at("my_server/owner/model_1"));
          ASSERT_NE(_models.end(), _models.find("my_server/owner/model_2"));
          EXPECT_EQ("model_2", _models.at("my_server/owner/model_2"));

          cv.notify_all();
        };

  fuelDB.Models(cb);

  // We now wait for a while unless the callback is executed and wake us up.
  std::unique_lock<std::mutex> lk(mutex);
  auto now = std::chrono::system_clock::now();
  EXPECT_TRUE(cv.wait_until(lk, now + std::chrono::milliseconds(100),
    [&cbExecuted]
    {
      return cbExecuted;
    }));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
