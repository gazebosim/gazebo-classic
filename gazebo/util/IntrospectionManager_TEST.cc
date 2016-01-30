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
  EXPECT_FALSE(intr->Register("item1", "type1", func));
  EXPECT_TRUE(intr->Unregister("item1"));
}

/////////////////////////////////////////////////
TEST(IntrospectionManagerTest, FilterManagement)
{
  std::vector<std::string> items;
  auto intr = util::IntrospectionManager::Instance();

  EXPECT_FALSE(intr->RemoveFilter("filter_#1"));
  EXPECT_FALSE(intr->Filter("filter_#1", items));
  intr->SetFilter("filter_#1", {"item1", "item2"});
  EXPECT_TRUE(intr->Filter("filter_#1", items));
  EXPECT_TRUE(intr->RemoveFilter("filter_#1"));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
