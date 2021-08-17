/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <functional>
#include <future>
#include <thread>
#include <gtest/gtest.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Event.hh>
#include "test/util.hh"

using namespace gazebo;

class EventTest : public gazebo::testing::AutoLogFixture { };

int g_callback = 0;
int g_callback1 = 0;
event::EventT<void ()> g_event;
event::ConnectionPtr g_conn;
event::ConnectionPtr g_conn2;

/////////////////////////////////////////////////
void callback()
{
  g_callback++;
}

/////////////////////////////////////////////////
void callback1()
{
  g_callback1++;
}

/////////////////////////////////////////////////
// Used by the CallbackDisconnect test.
void callbackDisconnect()
{
  // Remove both connections in the callback, which should not cause
  // a segfault.
  g_event.Disconnect(g_conn->Id());
  g_event.Disconnect(g_conn2->Id());
}

/////////////////////////////////////////////////
// Used by the CallbackDisconnect test.
void callbackDisconnect2()
{
  // This function should still be called, even though it was disconnected
  // in the callDisconnect function. The mutex in Event.hh prevents
  // a callback from deleting active connections until the event is
  // complete.
  ASSERT_TRUE(true);
}

/////////////////////////////////////////////////
// Make sure that calling disconnect in an event callback does not produce
// a segfault.
TEST_F(EventTest, CallbackDisconnect)
{
  // Create two connections
  g_conn = g_event.Connect(std::bind(&callbackDisconnect));
  g_conn2 = g_event.Connect(std::bind(&callbackDisconnect2));

  // Call the event. See the callback functions for more info.
  g_event();
}

/////////////////////////////////////////////////
TEST_F(EventTest, SignalOnce)
{
  g_callback = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  evt();

  EXPECT_EQ(g_callback, 1);
}

/////////////////////////////////////////////////
TEST_F(EventTest, SignalTwice)
{
  g_callback = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  evt();
  evt();

  EXPECT_EQ(g_callback, 2);
}

/////////////////////////////////////////////////
TEST_F(EventTest, SignalN)
{
  g_callback = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));

  for (unsigned int i = 0; i < 100; ++i)
    evt();

  EXPECT_EQ(g_callback, 100);
}

/////////////////////////////////////////////////
TEST_F(EventTest, Disconnect)
{
  g_callback = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));

  conn.reset();

  evt();

  EXPECT_EQ(g_callback, 0);
}

/////////////////////////////////////////////////
TEST_F(EventTest, MultiCallback)
{
  g_callback = 0;
  g_callback1 = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  event::ConnectionPtr conn1 = evt.Connect(std::bind(&callback1));

  evt();

  EXPECT_EQ(g_callback, 1);
  EXPECT_EQ(g_callback1, 1);
}

/////////////////////////////////////////////////
TEST_F(EventTest, MultiCallbackDisconnect)
{
  g_callback = 0;
  g_callback1 = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  event::ConnectionPtr conn1 = evt.Connect(std::bind(&callback1));
  conn.reset();

  evt();

  EXPECT_EQ(g_callback, 0);
  EXPECT_EQ(g_callback1, 1);
}

/////////////////////////////////////////////////
TEST_F(EventTest, MultiCallbackReconnect)
{
  g_callback = 0;
  g_callback1 = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  event::ConnectionPtr conn1 = evt.Connect(std::bind(&callback1));
  conn.reset();
  conn = evt.Connect(std::bind(&callback));

  evt();

  EXPECT_EQ(g_callback, 1);
  EXPECT_EQ(g_callback1, 1);
}

/////////////////////////////////////////////////
TEST_F(EventTest, ManyChanges)
{
  g_callback = 0;
  g_callback1 = 0;

  event::EventT<void ()> evt;
  event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
  event::ConnectionPtr conn1 = evt.Connect(std::bind(&callback1));
  conn.reset();
  conn1.reset();

  evt();

  EXPECT_EQ(g_callback, 0);
  EXPECT_EQ(g_callback1, 0);

  conn = evt.Connect(std::bind(&callback));
  conn1 = evt.Connect(std::bind(&callback1));

  evt();

  EXPECT_EQ(g_callback, 1);
  EXPECT_EQ(g_callback1, 1);

  evt();

  EXPECT_EQ(g_callback, 2);
  EXPECT_EQ(g_callback1, 2);

  conn1.reset();

  evt();

  EXPECT_EQ(g_callback, 3);
  EXPECT_EQ(g_callback1, 2);
}


/////////////////////////////////////////////////
// Race condition helper functions
void create_connections(event::EventT<void ()> & evt,
                        std::future<void> exit_obj_add,
                        std::list<event::ConnectionPtr> & connection_list)
{
  std::cout << "Start create connections thread" << std::endl;
  while(exit_obj_add.wait_for(std::chrono::milliseconds(1)) ==
        std::future_status::timeout) {
    event::ConnectionPtr conn = evt.Connect(std::bind(&callback));
    connection_list.push_front(conn);
  }
}

void access_connections(event::EventT<void ()> & evt,
                       std::future<void> exit_obj_add)
{
  std::cout << "Start access thread" << std::endl;
  while(exit_obj_add.wait_for(std::chrono::milliseconds(1)) ==
        std::future_status::timeout)
    // just calling the method without any other intent
    evt.ConnectionCount();
}


void remove_connections(event::EventT<void ()> & evt,
                        std::list<event::ConnectionPtr> & connection_list)
{
  std::cout << "Start remove thread" << std::endl;
  for (auto const& connection : connection_list) {
    evt.Disconnect(connection->Id());
  }
}

TEST_F(EventTest, RaceConditions)
{
  // Create three threads: 1) add_t to insert connections, 2) access_t to call
  // the count method 3) remove_t
  std::list<event::ConnectionPtr> connection_list;
  event::EventT<void ()> evt;
  std::promise<void> exit_signal;
  std::promise<void> exit_signal2;
  std::future<void> exit_obj_add = exit_signal.get_future();
  std::future<void> exit_obj_access = exit_signal2.get_future();
  // Run threads 1 and 2
  std::thread add_t(create_connections, std::ref(evt),
                    std::move(exit_obj_add),
                    std::ref(connection_list));
  std::thread access_t(access_connections, std::ref(evt),
                       std::move(exit_obj_access));
  // Give 1 second to adding and accessing threads and start the remove thread
  // leave all them working for 2 seconds.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::thread remove_t(remove_connections,
                      std::ref(evt),
                      std::ref(connection_list));
  std::this_thread::sleep_for(std::chrono::seconds(2));
  // Enable the event, call connection count and destroy it trying to make a
  // possible race condition to appear.
  evt();
  evt.ConnectionCount();
  evt.~EventT();

  exit_signal.set_value();
  add_t.join();
  exit_signal2.set_value();
  access_t.join();
  remove_t.join();
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
