/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

// certain tests fail (with the symbody engine
// setting this to true skips those tests
bool SKIP_FAILING_TESTS = true;

// this is the test fixture
class SimEventsTest : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  public: void SimPauseRun(const std::string &_physicsEngine);
  public: void SpawnAndDeleteModel(const std::string &_physicsEngine);
  public: void ModelInAndOutOfRegion(const std::string &_physicsEngine);
  public: void OccupiedEventSource(const std::string &_physicsEngine);
  public: void JointEventSource(const std::string &_physicsEngine);
};

// globals to exchange data between threads
boost::mutex g_mutex;
unsigned int g_event_count = 0;
std::string g_event_data;
std::string g_event_type;
std::string g_event_name;

// callback for SimEvent messages
// increment a counter and keep the data around
void ReceiveSimEvent(ConstSimEventPtr &_msg)
{
  boost::mutex::scoped_lock lock(g_mutex);
  g_event_count += 1;
  g_event_type = _msg->type();
  g_event_name = _msg->name();
  g_event_data = _msg->data();
  gzdbg << "ReceiveSimEvent " << g_event_type << std::endl;
}

// get the count in a thread safe way
unsigned int GetEventCount()
{
  boost::mutex::scoped_lock lock(g_mutex);
  return g_event_count;
}

// get the last event type in thread safe way
std::string GetEventType()
{
  boost::mutex::scoped_lock lock(g_mutex);
  return g_event_type;
}

// get the data in a thread safe way
std::string GetEventData()
{
  boost::mutex::scoped_lock lock(g_mutex);
  return g_event_data;
}

// waits for one or multiple events. if the expected number is
// specified, then the function can return early
unsigned int WaitForNewEvent(unsigned int current,
                             unsigned int max_tries = 50,
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
// SimPauseRun:
// Load test world, pause, run and verify that events are generated.
////////////////////////////////////////////////////////////////////////
void SimEventsTest::SimPauseRun(const std::string &_physicsEngine)
{
  Load("worlds/sim_events.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");

  // setup the callback that increments the counter everytime a
  // SimEvent is emitted.
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  // check that after pause, we have received a new event
  unsigned int count_before = GetEventCount();
  SetPause(true);
  unsigned int count_after = WaitForNewEvent(count_before);
  EXPECT_GT(count_after, count_before);
  // run the sim and check for event
  count_before = GetEventCount();
  SetPause(false);
  count_after = WaitForNewEvent(count_before);
  EXPECT_GT(count_after, count_before);
}

////////////////////////////////////////////////////////////////////////
// SpawnAndDeleteModel
// Load test world, add/delete models and verify that events are
// generated.
////////////////////////////////////////////////////////////////////////
void SimEventsTest::SpawnAndDeleteModel(const std::string &_physicsEngine)
{
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  Load("worlds/sim_events.world", false, _physicsEngine);
  // setup the callback that increments the counter everytime a
  // SimEvent is emitted.
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  unsigned int countBefore, countAfter;
  std::string name = "beer";

  countBefore = GetEventCount();
  std::string modelUri = "model://beer";
  SpawnModel(modelUri);
  countAfter = WaitForNewEvent(countBefore, 10, 100);
  EXPECT_GT(countAfter, countBefore);

  countBefore = GetEventCount();
  RemoveModel(name);
  countAfter = WaitForNewEvent(countBefore);
  EXPECT_GT(countAfter, countBefore);
  EXPECT_EQ(GetEventType(), "existence");
}


////////////////////////////////////////////////////////////////////////
// ModelInAndOutOfRegion:
// Load test world, move models and verify that events are generated.
////////////////////////////////////////////////////////////////////////
void SimEventsTest::ModelInAndOutOfRegion(const std::string &_physicsEngine)
{
  // simbody stepTo() failure
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  Load("worlds/sim_events.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  // setup the callback that increments the counter everytime a
  // SimEvent is emitted.
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  physics::ModelPtr can1 = world->GetModel("can1");
  EXPECT_TRUE(can1 != NULL);

  unsigned int countBefore1 = GetEventCount();
  can1->SetWorldPose(math::Pose(0, 5, 0, 0, 0, 0));
  unsigned int countAfter1 = WaitForNewEvent(countBefore1, 10, 100);
  EXPECT_GT(countAfter1, countBefore1);

  // move can1 into the end region
  unsigned int countBefore2 = GetEventCount();
  can1->SetWorldPose(math::Pose(10, 10, 0, 0, 0, 0));
  unsigned int countAfter2 = WaitForNewEvent(countBefore2, 10, 100);
  EXPECT_GT(countAfter2, countBefore2);
}

////////////////////////////////////////////////////////////////////////
// OccupiedEventSource:
// Load test world, move model and verify that events are generated by
// checking the position of the elevator.
////////////////////////////////////////////////////////////////////////
void SimEventsTest::OccupiedEventSource(const std::string &_physicsEngine)
{
  // simbody stepTo() failure
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  this->Load("worlds/elevator.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");

  // Get the elevator model
  physics::ModelPtr elevatorModel = world->GetModel("elevator");

  gzdbg << "Elevator Pose1["
        << elevatorModel->GetWorldPose().pos << "]\n";

  // Make sure the elevator is on the ground level
  EXPECT_LT(elevatorModel->GetWorldPose().pos.z, 0.08);
  EXPECT_GT(elevatorModel->GetWorldPose().pos.z, 0.07);

  // Spawn a box on the second floor, which should call the elevator up.
  this->SpawnBox("_my_test_box_", math::Vector3(0.5, 0.5, 0.5),
      math::Vector3(2, 0, 3.65), math::Vector3(0, 0, 0));

  // Wait for elevator to move. 10 seconds is more than long enough.
  common::Time::Sleep(10);

  gzdbg << "Elevator Pose2["
        << elevatorModel->GetWorldPose().pos << "]\n";

  // Make sure the elevator has moved up to the second floor.
  EXPECT_LT(elevatorModel->GetWorldPose().pos.z, 3.08);
  EXPECT_GT(elevatorModel->GetWorldPose().pos.z, 3.05);
}

////////////////////////////////////////////////////////////////////////
// JointEventSource:
// Load test world, rotate joint and verify that events are generated
////////////////////////////////////////////////////////////////////////
void SimEventsTest::JointEventSource(const std::string &_physicsEngine)
{
  // simbody stepTo() failure
  if (SKIP_FAILING_TESTS && _physicsEngine != "ode") return;

  this->Load("worlds/sim_events.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");

  // Get the elevator model
  physics::ModelPtr model = world->GetModel("revoluter");
  physics::JointPtr joint = model->GetJoint("joint");

  // setup the callback that increments the counter everytime a
  // SimEvent is emitted.
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::SubscriberPtr sceneSub = node->Subscribe("/gazebo/sim_events",
      &ReceiveSimEvent);

  // check that after position, we have received a new event
  unsigned int count_before = GetEventCount();
  // rotate joint

  joint->SetPosition(0, IGN_PI);
  // check for event
  unsigned int count_after = WaitForNewEvent(count_before);
  EXPECT_GT(count_after, count_before);

  count_before = GetEventCount();

  joint->SetVelocity(0, 3.1);
  // check for event
  count_after = WaitForNewEvent(count_before);
  EXPECT_GT(count_after, count_before);
}


// Run all test cases
INSTANTIATE_TEST_CASE_P(PhysicsEngines, SimEventsTest, PHYSICS_ENGINE_VALUES);

TEST_P(SimEventsTest, JointEventSource)
{
  JointEventSource(GetParam());
}

TEST_P(SimEventsTest, SimPauseRun)
{
  SimPauseRun(GetParam());
}

TEST_P(SimEventsTest, SpawnAndDeleteModel)
{
  SpawnAndDeleteModel(GetParam());
}

TEST_P(SimEventsTest, ModelInAnOutOfRegion)
{
  ModelInAndOutOfRegion(GetParam());
}

TEST_P(SimEventsTest, OccupiedEventSource)
{
  OccupiedEventSource(GetParam());
}

// main, where we can specify to skip certain tests
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
