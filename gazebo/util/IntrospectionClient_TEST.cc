/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <set>
#include <string>
#include <ignition/transport.hh>
#include <gtest/gtest.h>
#include "gazebo/common/Exception.hh"
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionClient.hh"
#include "gazebo/util/IntrospectionManager.hh"
#include "test/util.hh"

using namespace gazebo;

/// \brief A test fixture class.
class IntrospectionClientTest : public ::testing::Test
{
  /// \brief Constructor
  public: IntrospectionClientTest() :
    callbackExecuted(false),
    manager(util::IntrospectionManager::Instance())
  {
    this->managerId = this->manager->Id();
  }

  /// \brief Function called each time a topic update is received.
  public: void MyCb(const gazebo::msgs::Param_V &_msg)
  {
    ASSERT_EQ(_msg.param_size(), 2);
    std::map<std::string, double> receivedItems;

    for (auto i = 0; i < _msg.param_size(); ++i)
    {
      auto param = _msg.param(i);
      ASSERT_TRUE(param.has_value());
      auto value = param.value();
      EXPECT_EQ(value.type(), gazebo::msgs::Any::DOUBLE);
      ASSERT_TRUE(value.has_double_value());
      receivedItems[param.name()] = value.double_value();
    }

    ASSERT_TRUE(receivedItems.find("item1") != receivedItems.end());
    EXPECT_DOUBLE_EQ(receivedItems.at("item1"), 1.0);

    ASSERT_TRUE(receivedItems.find("item2") != receivedItems.end());
    EXPECT_DOUBLE_EQ(receivedItems.at("item2"), 1.0);

    this->callbackExecuted = true;
  }

  /// \brief Subscribe to a given topic.
  public: void Subscribe(const std::string &_topic)
  {
    if (this->node.Subscribe(_topic, &IntrospectionClientTest::MyCb, this))
      this->topicsSubscribed.emplace(_topic);
  }

  /// \brief Initialize the test.
  public: void SetUp()
  {
    this->topicsSubscribed = {};

    // A callback for updating items.
    auto func = []()
    {
      return 1.0;
    };

    // Make sure that we always have some items registered.
    this->manager->Register<double>("item1", func);
    this->manager->Register<double>("item2", func);
    this->manager->Register<double>("item3", func);

    this->callbackExecuted = false;
  }

  /// \brief Executed at the end of each test.
  public: void TearDown()
  {
    // Make sure that we unsubscribe from all the topics.
    for (auto const &topic : this->topicsSubscribed)
      this->node.Unsubscribe(topic);
  }

  /// \brief Flag to detect whether the callback was executed.
  protected: bool callbackExecuted;

  /// \brief Pointer to the introspection manager.
  protected: util::IntrospectionManager *manager;

  /// \brief The ID of the manager.
  protected: std::string managerId;

  /// \brief An introspection client.
  protected: util::IntrospectionClient client;

  /// \brief Transport node.
  private: ignition::transport::Node node;

  /// \brief The set of topics subscribed.
  private: std::set<std::string> topicsSubscribed;
};

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, Managers)
{
  // List of introspection managers. We should have at least one.
  std::set<std::string> managerIds;
  managerIds = this->client.Managers();
  EXPECT_GT(managerIds.size(), 0u);
  EXPECT_TRUE(managerIds.find(this->managerId) != managerIds.end());
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, NewAndRemoveFilter)
{
  std::string filterId;
  std::string topic;

  // Try to create an empty filter.
  std::set<std::string> emptySet;
  EXPECT_FALSE(this->client.NewFilter(this->managerId, emptySet, filterId,
      topic));

  // Let's create a filter for receiving updates on "item1" and "item2".
  std::set<std::string> items = {"item1", "item2"};
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, filterId, topic));

  // Try to create a filter using an incorrect manager ID.
  EXPECT_FALSE(this->client.NewFilter("_wrong_id_", items, filterId, topic));

  // Subscribe to my custom topic for receiving updates.
  this->Subscribe(topic);

  // Trigger an update.
  this->manager->Update();

  // Check that we received the update.
  EXPECT_TRUE(this->callbackExecuted);
  this->callbackExecuted = false;

  // Try to remove a filter with the wrong manager ID.
  EXPECT_FALSE(this->client.RemoveFilter("_wrong_id_", filterId));

  // Try to remove a filter with the wrong filter ID.
  EXPECT_FALSE(this->client.RemoveFilter(this->managerId, "_wrong_id_"));

  // Remove an existing filter.
  EXPECT_TRUE(this->client.RemoveFilter(this->managerId, filterId));

  // Trigger an update to verify that we don't receive any updates.
  this->manager->Update();

  // Check that we didn't receive any updates, we shouldn't have filters.
  EXPECT_FALSE(this->callbackExecuted);
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, RemoveAllFilters)
{
  std::string filterId1;
  std::string filterId2;
  std::string topic;

  // Let's create a filter for receiving updates on "item1" and "item2".
  std::set<std::string> items = {"item1", "item2"};
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, filterId1, topic));

  // Subscribe to my custom topic for receiving updates.
  this->Subscribe(topic);

  // Let's create another filter for receiving updates on "item1" and "item2".
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, filterId2, topic));

  // Subscribe to my custom topic for receiving updates.
  this->Subscribe(topic);

  // Trigger an update.
  this->manager->Update();

  // Check that we received the update.
  EXPECT_TRUE(this->callbackExecuted);
  this->callbackExecuted = false;

  // Remove all filters.
  EXPECT_TRUE(this->client.RemoveAllFilters());

  // Trigger an update to verify that we don't receive any updates.
  this->manager->Update();

  // Check that we didn't receive any updates, we shouldn't have filters.
  EXPECT_FALSE(this->callbackExecuted);
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, NewFilterAsync)
{
  bool executed = false;
  auto cb = [&](const std::string &/*_filterId*/,
               const std::string &/*_newTopic*/,
               const bool _result)
  {
    executed = true;
    EXPECT_FALSE(_result);
  };

  // Try to create an empty filter.
  std::set<std::string> emptySet;
  EXPECT_FALSE(this->client.NewFilter(this->managerId, emptySet, cb));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(executed);

  auto cb2 = [this](const std::string &/*_filterId*/,
                 const std::string &_newTopic,
                 const bool _result)
  {
    // Subscribe to my custom topic for receiving updates.
    this->Subscribe(_newTopic);

    EXPECT_TRUE(!_newTopic.empty());
    EXPECT_TRUE(_result);
  };

  // Let's create a filter for receiving updates on "item1" and "item2".
  std::set<std::string> items = {"item1", "item2"};
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, cb2));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Trigger an update.
  this->manager->Update();

  // Check that we received the update.
  EXPECT_TRUE(this->callbackExecuted);
  this->callbackExecuted = false;
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, UpdateFilter)
{
  // Let's create a filter.
  std::set<std::string> items = {"item1", "item3"};
  std::string filterId;
  std::string topic;
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, filterId, topic));

  // Update the filter. We're interested on "item1" and "item2".
  items = {"item1", "item2"};
  EXPECT_TRUE(this->client.UpdateFilter(this->managerId, filterId, items));

  // Try to update a filter with an incorrect manager ID.
  EXPECT_FALSE(this->client.UpdateFilter("_wrong_id_", filterId, items));

  // Try to update a filter wih an incorrect filter ID.
  EXPECT_FALSE(this->client.UpdateFilter(this->managerId, "_wrong_id_", items));

  // Subscribe to my custom topic for receiving updates.
  this->Subscribe(topic);

  // Trigger an update.
  this->manager->Update();

  // Check that we received the update.
  EXPECT_TRUE(this->callbackExecuted);
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, UpdateFilterAsync)
{
  std::string filterId;
  bool executed = false;

  auto cbNew = [&](const std::string &_filterId,
                   const std::string &_newTopic,
                   const bool _result)
  {
    executed = true;
    EXPECT_TRUE(_result);

    // Subscribe to my custom topic for receiving updates.
    this->Subscribe(_newTopic);

    filterId = _filterId;
  };

  // Let's create a filter.
  std::set<std::string> items = {"item1", "item3"};

  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, cbNew));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);
  executed = false;

  auto cbUpdateWrong = [&](const bool _result)
  {
    executed = true;
    EXPECT_FALSE(_result);
  };

  // Try to update a filter with an incorrect manager ID.
  EXPECT_FALSE(this->client.UpdateFilter("_wrong_id_", filterId, items,
    cbUpdateWrong));
  executed = false;

  // Try to update a filter wih an incorrect filter ID.
  EXPECT_TRUE(this->client.UpdateFilter(this->managerId, "_wrong_id_",
    items, cbUpdateWrong));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);
  executed = false;

  auto cbUpdate = [&](const bool _result)
  {
    executed = true;
    EXPECT_TRUE(_result);
  };

  // Update the filter. We're interested on "item1" and "item2".
  items = {"item1", "item2"};
  EXPECT_TRUE(this->client.UpdateFilter(this->managerId, filterId, items,
    cbUpdate));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Trigger an update.
  this->manager->Update();

  EXPECT_TRUE(executed);

  // Check that we received the update.
  EXPECT_TRUE(this->callbackExecuted);
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, RemoveFilterAsync)
{
  std::string filterId;
  bool executed = false;

  auto cbNew = [&](const std::string &_filterId,
                   const std::string &_newTopic,
                   const bool _result)
  {
    executed = true;
    EXPECT_TRUE(_result);

    // Subscribe to my custom topic for receiving updates.
    this->Subscribe(_newTopic);

    filterId = _filterId;
  };

  // Let's create a filter.
  std::set<std::string> items = {"item1", "item3"};

  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, cbNew));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);
  executed = false;

  auto cbRemoveBad = [&](const bool _result)
  {
    executed = true;
    EXPECT_FALSE(_result);
  };

  // Try to remove a filter with an incorrect manager ID.
  EXPECT_FALSE(this->client.RemoveFilter("_wrong_id_", filterId, cbRemoveBad));
  executed = false;

  // Try to remove a filter with the wrong filter ID.
  EXPECT_TRUE(this->client.RemoveFilter(this->managerId, "_wrong_id_",
    cbRemoveBad));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);
  executed = false;

  auto cbRemove = [&](const bool _result)
  {
    executed = true;
    EXPECT_TRUE(_result);
  };

  // Remove an existing filter.
  EXPECT_TRUE(this->client.RemoveFilter(this->managerId, filterId,
    cbRemove));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);

  // Trigger an update to verify that we don't receive any updates.
  this->manager->Update();

  // Check that we didn't receive any updates, we shouldn't have filters.
  EXPECT_FALSE(this->callbackExecuted);
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, Items)
{
  std::set<std::string> items;

  EXPECT_FALSE(this->client.IsRegistered("_wrong_id_", items));

  // Try to query the list of items with an incorrect manager ID.
  EXPECT_FALSE(this->client.Items("_wrong_id_", items));

  // Let's query the list of items available.
  EXPECT_TRUE(this->client.Items(this->managerId, items));
  EXPECT_TRUE(this->client.IsRegistered(this->managerId, "item1"));
  EXPECT_TRUE(this->client.IsRegistered(this->managerId,
        std::set<std::string> {"item1", "item2"}));
  EXPECT_EQ(items.size(), 3u);
  EXPECT_TRUE(items.find("item1") != items.end());
  EXPECT_TRUE(items.find("item2") != items.end());
  EXPECT_TRUE(items.find("item3") != items.end());
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, ItemsAsync)
{
  std::set<std::string> items;
  bool executed = false;

  EXPECT_FALSE(this->client.IsRegistered("_wrong_id_", items));

  auto cbItems = [&](const std::set<std::string> &_items,
                     const bool _result)
  {
    executed = true;
    EXPECT_TRUE(_result);

    EXPECT_EQ(_items.size(), 3u);
    EXPECT_TRUE(_items.find("item1") != _items.end());
    EXPECT_TRUE(_items.find("item2") != _items.end());
    EXPECT_TRUE(_items.find("item3") != _items.end());
  };

  // Let's query the list of items available.
  EXPECT_TRUE(this->client.Items(this->managerId, cbItems));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(executed);

  EXPECT_TRUE(this->client.IsRegistered(this->managerId, "item1"));
  EXPECT_TRUE(this->client.IsRegistered(this->managerId,
        std::set<std::string> {"item1", "item2"}));
}

/////////////////////////////////////////////////
TEST_F(IntrospectionClientTest, Exception)
{
  // A callback for updating items that triggers an exception.
  auto func = []()
  {
    gzthrow("Simulating an exception in user callback");
    return 1.0;
  };

  // Register a new item with the callback that raises exceptions.
  EXPECT_TRUE(this->manager->Register<double>("item4", func));

  // Let's create a filter.
  std::set<std::string> items = {"item4"};
  std::string filterId;
  std::string topic;
  EXPECT_TRUE(this->client.NewFilter(this->managerId, items, filterId, topic));

  // Subscribe to my custom topic for receiving updates.
  this->Subscribe(topic);

  // Trigger an update.
  this->manager->Update();

  // Check that we didn't received the update.
  EXPECT_FALSE(this->callbackExecuted);

  EXPECT_TRUE(this->manager->Unregister("item4"));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
