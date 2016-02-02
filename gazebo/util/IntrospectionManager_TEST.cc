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

#include <gtest/gtest.h>
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionManager.hh"
#include "test/util.hh"

using namespace gazebo;

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
  EXPECT_EQ(intr->RegisteredItems().size(), 1u);
  intr->Show();
  EXPECT_FALSE(intr->Register("item1", "type1", func));
  EXPECT_TRUE(intr->Unregister("item1"));
  intr->Show();
  EXPECT_TRUE(intr->RegisteredItems().empty());

  EXPECT_TRUE(intr->Register("item1", "type1", func));
  intr->Show();
  EXPECT_TRUE(intr->Register("item2", "type2", func));
  intr->Show();
  EXPECT_TRUE(intr->Register("item3", "type3", func));
  intr->Show();
  EXPECT_EQ(intr->RegisteredItems().size(), 3u);

  intr->Unregister("item1");
  intr->Unregister("item2");
  intr->Unregister("item3");
}

/////////////////////////////////////////////////
TEST(IntrospectionManagerTest, FilterManagement)
{
  std::string filterId1, filterId2, filterId3;
  std::set<std::string> items;
  auto intr = util::IntrospectionManager::Instance();

  //gazebo::msgs::Param_V req;
  //gazebo::msgs::GzString rep;
  //bool result;

  //auto nextParam = req.add_param();
  //nextParam->set_name("filter_id");
  //nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
  //nextParam->mutable_value()->set_string_value("filter_#1");

  //intr->RemoveFilter(req, rep, result);
  //EXPECT_FALSE(result);

  //req.Clear();
  //nextParam = req.add_param();
  //nextParam->set_name("item");
  //nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
  //nextParam->mutable_value()->set_string_value("item1");
  //nextParam = req.add_param();
  //nextParam->set_name("item");
  //nextParam->mutable_value()->set_type(gazebo::msgs::Any::STRING);
  //nextParam->mutable_value()->set_string_value("item2");

  //intr->NewFilter(req, rep, result);
  //EXPECT_TRUE(result);
  //intr->Show();

  //auto func = [](gazebo::msgs::Any &_msg)
  //{
  //  _msg.set_type(gazebo::msgs::Any::DOUBLE);
  //  _msg.set_double_value(1.0);
  //  return true;
  //};

  //EXPECT_TRUE(intr->Register("item1", "type1", func));
  //intr->Show();

  //intr->Update();

  EXPECT_FALSE(intr->Filter("filter_#1", items));
  EXPECT_TRUE(intr->NewFilter({"item1", "item2"}, filterId1));
  intr->Show();
  EXPECT_TRUE(intr->Filter(filterId1, items));
  EXPECT_EQ(items.size(), 2u);
  EXPECT_TRUE(intr->RemoveFilter(filterId1));
  intr->Show();

  EXPECT_TRUE(intr->NewFilter({"item1", "item2"}, filterId1));
  intr->Show();
  EXPECT_TRUE(intr->NewFilter({"item2", "item3"}, filterId2));
  intr->Show();
  EXPECT_TRUE(intr->NewFilter({"item1", "item4"}, filterId3));
  intr->Show();
  EXPECT_TRUE(intr->UpdateFilter(filterId1, {"item1", "item5"}));
  intr->Show();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
