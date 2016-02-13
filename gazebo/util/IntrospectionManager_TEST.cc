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

#include <ignition/math/Pose3.hh>
#include <gtest/gtest.h>
#include "gazebo/msgs/any.pb.h"
#include "gazebo/util/IntrospectionManager.hh"
#include "test/util.hh"

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
    // A callback for updating items.
    auto func = [](){return 1.0;};

    // Make sure that we always have some items registered.
    this->manager->Register<double>("item1", func);
    this->manager->Register<double>("item2", func);
    this->manager->Register<double>("item3", func);
  }

  /// \brief Pointer to the introspection manager.
  protected: util::IntrospectionManager *manager;
};

/////////////////////////////////////////////////
TEST_F(IntrospectionManagerTest, Id)
{
  EXPECT_TRUE(!this->manager->Id().empty());
}

/////////////////////////////////////////////////
TEST_F(IntrospectionManagerTest, Registration)
{
  // A callback for updating items.
  auto func = [](){return 2.0;};
  auto func2 = [](){return ignition::math::Pose3d();};

  EXPECT_TRUE(this->manager->Register<double>("item4", func));
  EXPECT_TRUE(this->manager->Register<ignition::math::Pose3d>("item5",func2));

  EXPECT_TRUE(this->manager->Unregister("item4"));
  EXPECT_TRUE(this->manager->Unregister("item5"));
}

/////////////////////////////////////////////////
TEST_F(IntrospectionManagerTest, RegistrationAndItems)
{
  // A callback for updating items.
  auto func = [](){return 1.0;};

  // Try to unregister an unregistered item.
  EXPECT_FALSE(this->manager->Unregister("_unregistered_item_"));

  // Register one more item.
  EXPECT_TRUE(this->manager->Register<double>("item4", func));
  EXPECT_EQ(this->manager->Items().size(), 4u);

  // Try to register an existing item.
  EXPECT_FALSE(this->manager->Register<double>("item4", func));

  // Unegister an existing item.
  EXPECT_TRUE(this->manager->Unregister("item4"));
  EXPECT_TRUE(!this->manager->Items().empty());

  // Unregister multiple items.
  EXPECT_TRUE(this->manager->Unregister("item1"));
  EXPECT_TRUE(this->manager->Unregister("item2"));
  EXPECT_TRUE(this->manager->Unregister("item3"));
  EXPECT_TRUE(this->manager->Items().empty());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
