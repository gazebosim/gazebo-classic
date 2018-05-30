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
#include "gazebo/common/Event.hh"
#include "gazebo/common/WeakBind.hh"
#include "test/util.hh"

using namespace gazebo;

class WeakBindTest : public gazebo::testing::AutoLogFixture { };

// Flag to check if event was received
bool g_eventReceived = false;

/// \brief Class for testing a weak bind to an event
class EventBindTestClass
    : public boost::enable_shared_from_this<EventBindTestClass>
{
  /// \brief Connect to event
  /// \param[in] _event Event to connect to
  public: void Connect(event::EventT<void()> &_event)
    {
      // Test fails if we use boost::bind
      this->connection = _event.Connect(common::weakBind(
          &EventBindTestClass::OnEvent, this->shared_from_this()));
    }

  /// \brief Event callback
  private: void OnEvent()
    {
      g_eventReceived = true;
    }

  /// \brief Keep a pointer to the connection alive
  private: event::ConnectionPtr connection;
};

/////////////////////////////////////////////////
TEST_F(WeakBindTest, EventBind)
{
  // Create an event
  event::EventT<void ()> event;

  // Create test object
  boost::shared_ptr<EventBindTestClass> testObj(new EventBindTestClass());
  ASSERT_NE(nullptr, testObj);
  testObj->Connect(event);

  // Trigger event and check callback was called
  event();
  EXPECT_TRUE(g_eventReceived);

  // Destroy object and trigger event again
  g_eventReceived = false;
  testObj.reset();

  event();
  EXPECT_FALSE(g_eventReceived);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
