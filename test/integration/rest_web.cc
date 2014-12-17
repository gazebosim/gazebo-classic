/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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


#include <limits>

#include "ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

// #include "gazebo/physics/physics.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

// certain tests fail (with the symbody engine
// setting this to true skips those tests
bool SKIP_FAILING_TESTS = true;

// this is the test fixture
class RestWebTest : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  public: void FirstTest(const std::string &_physicsEngine);
};

// globals to exchange data between threads
boost::mutex g_mutex;
unsigned int g_count;

// RestLogin: string url, username, password
// RestError: string type, msg
// RestPost:  string route, json


// callback for SimEvent messages
// increment a counter and keep the data around
void ReceiveRestError(ConstRestErrorPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_count += 1;
}

// get the count in a thread safe way
unsigned int GetErrorCount()
{
  boost::mutex::scoped_lock lock(g_mutex);
  return g_count;
}

// get the last event type in thread safe way
std::string LastRestError()
{
  boost::mutex::scoped_lock lock(g_mutex);
  return "";
}


// waits for one or multiple events. if the expected number is
// specified, then the function can return early
unsigned int WaitForNewError(unsigned int current,
                             unsigned int max_tries = 10,
                             unsigned int ms = 10)
{
  for (unsigned int i = 0; i < max_tries; i++)
  {
    unsigned int count = GetErrorCount();
    if (count > current)
    {
      return count;
    }
    common::Time::MSleep(ms);
  }
  return GetErrorCount();
}

// test macro
TEST_P(RestWebTest, FirstTest)
{
  FirstTest(GetParam());
}

////////////////////////////////////////////////////////////////////////
// SimPauseRun:
// Load test world, publish login and post messages
////////////////////////////////////////////////////////////////////////
void RestWebTest::FirstTest(const std::string &_physicsEngine)
{
  char *argv[] = {"program name", "-s", "libRestWebPlugin.so"}; 
  //char **v = {"-s", "libRestWebPlugin.so"};
  Load("test/worlds/rest_web.world", false, _physicsEngine, 3, argv);
  physics::WorldPtr world = physics::get_world("default");

  // setup the callback that increments a counter each time a
  // RestError is emitted.
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/rest_error",
      &ReceiveRestError);

  // check that after pause, we have received a new event
  unsigned int count_before = GetErrorCount();

  // publish to the login topic

  unsigned int count_after = WaitForNewError(count_before);
  EXPECT_GT(count_after, count_before);
  // run the sim and check for event
  count_before = GetErrorCount();
  SetPause(false);
  count_after = WaitForNewError(count_before);
  EXPECT_GT(count_after, count_before);
}

// magic macro
INSTANTIATE_TEST_CASE_P(PhysicsEngines, RestWebTest, PHYSICS_ENGINE_VALUES);

// main, where we can specify to skip certain tests
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
