/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

// certain tests fail (with the symbody engine
// setting this to true skips those tests
bool SKIP_FAILING_TESTS = true;

// this is the test fixture
class RegionEventTest
    : public ServerFixture, public testing::WithParamInterface<const char*>
{
  public: void ModelEnteringRegion(const std::string &_physicsEngine);
  public: void ModelLeavingRegion(const std::string &_physicsEngine);
};

// globals to exchange data between threads
boost::mutex g_mutex;
unsigned int g_event_count = 0;
std::string g_event_data;
std::string g_event_type;
std::string g_event_name;

/////////////////////////////////////////////////
// callback for SimEvent messages
// increment a counter and keep the data around
void ReceiveSimEvent(ConstSimEventPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  {
    g_event_count += 1;
    g_event_type = _msg->type();
    g_event_name = _msg->name();
    g_event_data = _msg->data();
  }
}

/////////////////////////////////////////////////
unsigned int GetEventCount()
{
  boost::mutex::scoped_lock lock(g_mutex);
  {
    return g_event_count;
  }
}

/////////////////////////////////////////////////
std::string GetEventType()
{
  boost::mutex::scoped_lock lock(g_mutex);
  {
    return g_event_type;
  }
}

/////////////////////////////////////////////////
std::string GetEventName()
{
  boost::mutex::scoped_lock lock(g_mutex);
  {
    return g_event_name;
  }
}

/////////////////////////////////////////////////
std::string GetEventData()
{
  boost::mutex::scoped_lock lock(g_mutex);
  {
    return g_event_data;
  }
}

/////////////////////////////////////////////////
// waits for one or multiple events. if the expected number is
// specified, then the function can return early
unsigned int WaitForNewEvent(unsigned int current,
                             unsigned int max_tries = 10,
                             unsigned int ms = 10)
{
  for (unsigned int i = 0; i < max_tries; i++)
  {
    unsigned int count = GetEventCount();
    if (count > current)
    {
      return count;
    }
    common::Time::MSleep(ms);
  }
  return GetEventCount();
}

////////////////////////////////////////////////////////////////////////
// ModelEnteringRegion:
// Load test world, move model into event region and verify event.
////////////////////////////////////////////////////////////////////////
void RegionEventTest::ModelEnteringRegion(const std::string &_physicsEngine)
{
  // simbody stepTo() failure
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  Load("test/worlds/region_events.world", false, _physicsEngine);

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();

  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  physics::ModelPtr regionEventBox = world->GetModel("RegionEventBox");
  ASSERT_TRUE(regionEventBox != NULL);

  physics::ModelPtr boxModel = world->GetModel("box");
  ASSERT_TRUE(boxModel != NULL);

  unsigned int startingCount = GetEventCount();
  boxModel->SetWorldPose(regionEventBox->GetWorldPose());
  unsigned int endingCount = WaitForNewEvent(startingCount, 10, 100);

  ASSERT_GT(endingCount, startingCount);
  ASSERT_STREQ("inclusion", GetEventType().c_str());
  ASSERT_STREQ("model_in_region_event_box", GetEventName().c_str());
}

////////////////////////////////////////////////////////////////////////
// ModelLeavingRegion:
// Load test world, move model outside of event region and verify event.
////////////////////////////////////////////////////////////////////////
void RegionEventTest::ModelLeavingRegion(const std::string &_physicsEngine)
{
  // simbody stepTo() failure
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  Load("test/worlds/region_events.world", false, _physicsEngine);

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();

  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  physics::ModelPtr regionEventBox = world->GetModel("RegionEventBox");
  ASSERT_TRUE(regionEventBox != NULL);

  physics::ModelPtr boxModel = world->GetModel("box");
  ASSERT_TRUE(boxModel != NULL);

  math::Pose regionEventBoxPos = regionEventBox->GetWorldPose();
  math::Pose boxModelPose = boxModel->GetWorldPose();

  boxModel->SetWorldPose(regionEventBox->GetWorldPose());
  (void) WaitForNewEvent(GetEventCount(), 10, 100);

  unsigned int startingCount = GetEventCount();
  {
    math::Pose newPose = regionEventBox->GetWorldPose();
    newPose.pos.x += 5.0;
    newPose.pos.y += 5.0;
    boxModel->SetWorldPose(newPose);
  }
  unsigned int endingCount = WaitForNewEvent(startingCount, 10, 100);

  EXPECT_GT(endingCount, startingCount);
  EXPECT_STREQ("inclusion", GetEventType().c_str());
  EXPECT_STREQ("model_in_region_event_box", GetEventName().c_str());
}

/////////////////////////////////////////////////
TEST_P(RegionEventTest, ModelEnteringRegion)
{
  ModelEnteringRegion(GetParam());
}

/////////////////////////////////////////////////
TEST_P(RegionEventTest, ModelLeavingRegion)
{
  ModelLeavingRegion(GetParam());
}

// Run all test cases
INSTANTIATE_TEST_CASE_P(PhysicsEngines, RegionEventTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc > 1)
  {
    std::string skipStr = argv[1];
    if (skipStr == "no_skip")
    {
      SKIP_FAILING_TESTS = false;
    }
  }
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
