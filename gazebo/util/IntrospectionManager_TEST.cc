/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <ignition/transport.hh>
#include <gtest/gtest.h>
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionManager.hh"
#include "test/util.hh"

using namespace gazebo;

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void myCb(const gazebo::msgs::Param_V &_msg)
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
}

/////////////////////////////////////////////////
TEST(IntrospectionManagerTest, Registration)
{
  auto intr = util::IntrospectionManager::Instance();

  auto func = [](gazebo::msgs::Any &_msg)
  {
    _msg.set_type(gazebo::msgs::Any::DOUBLE);
    _msg.set_double_value(1.0);
    return true;
  };

  EXPECT_FALSE(intr->Unregister("item1"));
  EXPECT_TRUE(intr->Register("item1", "type1", func));
  EXPECT_EQ(intr->Items().size(), 1u);
  EXPECT_FALSE(intr->Register("item1", "type1", func));
  EXPECT_TRUE(intr->Unregister("item1"));
  EXPECT_TRUE(intr->Items().empty());

  EXPECT_TRUE(intr->Register("item1", "type1", func));
  EXPECT_TRUE(intr->Register("item2", "type2", func));
  EXPECT_TRUE(intr->Register("item3", "type3", func));
  EXPECT_EQ(intr->Items().size(), 3u);

  intr->Unregister("item1");
  intr->Unregister("item2");
  intr->Unregister("item3");
}

/////////////////////////////////////////////////
TEST(IntrospectionManagerTest, FiltersWithFunctions)
{
  std::string filterId1, filterId2, filterId3;
  std::set<std::string> items;
  auto intr = util::IntrospectionManager::Instance();

  EXPECT_FALSE(intr->RemoveFilter("unknownId"));

  EXPECT_TRUE(intr->NewFilter({"item1", "item2"}, filterId1));
  EXPECT_TRUE(intr->RemoveFilter(filterId1));

  EXPECT_TRUE(intr->NewFilter({"item1", "item2"}, filterId1));
  EXPECT_TRUE(intr->NewFilter({"item2", "item3"}, filterId2));
  EXPECT_TRUE(intr->NewFilter({"item1", "item4"}, filterId3));
  EXPECT_TRUE(intr->UpdateFilter(filterId1, {"item1", "item5"}));

  EXPECT_TRUE(intr->RemoveFilter(filterId1));
  EXPECT_TRUE(intr->RemoveFilter(filterId2));
  EXPECT_TRUE(intr->RemoveFilter(filterId3));
}

/////////////////////////////////////////////////
TEST(IntrospectionManagerTest, FiltersWithServices)
{
  bool result;
  unsigned int timeout = 500;
  std::string filterId1, filterId2, filterId3;
  std::string myTopic;
  std::set<std::string> items;
  auto intr = util::IntrospectionManager::Instance();

  // A callback.
  auto func = [](gazebo::msgs::Any &_msg)
  {
    _msg.set_type(gazebo::msgs::Any::DOUBLE);
    _msg.set_double_value(1.0);
    return true;
  };

  // Register some items.
  EXPECT_TRUE(intr->Register("item1", "type1", func));
  EXPECT_TRUE(intr->Register("item2", "type2", func));
  EXPECT_TRUE(intr->Register("item3", "type3", func));

  // Simulate a client interacting with the introspection service via services.
  ignition::transport::Node node;

  // First, get the list of services available.
  std::vector<std::string> availableServices;
  node.ServiceList(availableServices);

  // We should have one service named /introspection/<id>/items . Let's find it.
  std::string itemsService;
  bool found = false;
  const std::string kStartDelim = "/introspection/";
  const std::string kEndDelim = "/items";
  for (auto service : availableServices)
  {
    itemsService = service;
    if ((service.find(kStartDelim) == 0) &&
        (service.rfind(kEndDelim)) == service.size() - kEndDelim.size())
    {
      found = true;
      break;
    }
  }

  ASSERT_TRUE(found);

  // Save the introspection manager ID.
  auto end = itemsService.rfind(kEndDelim);
  std::string managerId = itemsService.substr(kStartDelim.size(),
      end - kStartDelim.size());

  // Let's use the "items" service for querying the list of items available.
  {
    gazebo::msgs::Empty req;
    gazebo::msgs::Param_V rep;

    // Request the "/introspection/<id>/items" service.
    ASSERT_TRUE(node.Request(itemsService, req, timeout, rep, result));
    ASSERT_TRUE(result);

    // Let's save all the items received in a set.
    std::set<std::string> itemsReceived;
    for (auto i = 0; i < rep.param_size(); ++i)
    {
      auto param = rep.param(i);
      EXPECT_EQ(param.name(), "item");
      ASSERT_TRUE(param.has_value());
      auto value = param.value();
      EXPECT_EQ(value.type(), gazebo::msgs::Any::STRING);
      ASSERT_TRUE(value.has_string_value());
      itemsReceived.emplace(value.string_value());
    }

    EXPECT_TRUE(itemsReceived.find("item1") != itemsReceived.end());
    EXPECT_TRUE(itemsReceived.find("item2") != itemsReceived.end());
    EXPECT_TRUE(itemsReceived.find("item3") != itemsReceived.end());
  }

  // Let's create a filter for receiving updates on "item1" and "item2".
  {
    gazebo::msgs::Param_V req;
    gazebo::msgs::GzString rep;

    // Request the "/introspection/<id>/filter_new" service.
    auto newFilterService = "/introspection/" + managerId + "/filter_new";
    // Add to the message the list of items to include in the filter.
    auto nextParam = req.add_param();
    nextParam->set_name("item");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
    nextParam->mutable_value()->set_string_value("item1");
    nextParam = req.add_param();
    nextParam->set_name("item");
    nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
    nextParam->mutable_value()->set_string_value("item2");

    // Set the new filter.
    ASSERT_TRUE(node.Request(newFilterService, req, timeout, rep, result));
    ASSERT_TRUE(result);
    EXPECT_TRUE(!rep.data().empty());

    // Let's save the topic for receiving all the future updates.
    myTopic = "/introspection/filter/" + rep.data();
  }

  // Let's subscribe to my custom topic for receiving updates.
  node.Subscribe(myTopic, myCb);

  // Trigger an update.
  intr->Update();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
