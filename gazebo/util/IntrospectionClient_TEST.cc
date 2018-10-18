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
    auto func = [](gazebo::msgs::Any &_msg)
    {
      _msg.set_type(gazebo::msgs::Any::DOUBLE);
      _msg.set_double_value(1.0);
      return true;
    };

    // Make sure that we always have some items registered.
    this->manager->Register("item1", "type1", func);
    this->manager->Register("item2", "type2", func);
    this->manager->Register("item3", "type3", func);

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
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
