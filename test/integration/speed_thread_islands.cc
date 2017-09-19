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
#include "gazebo/common/Timer.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
const double g_tolerance = 1e-2;

class SpeedThreadIslandsTest : public ServerFixture,
                               public testing::WithParamInterface<const char*>
{
  /// \brief Load 8 double pendulums arranged in a circle.
  /// Unthrottle update rate, set island threads, and check
  /// changes in required computational time.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _solverType Type of solver to use.
  public: void RevoluteJoint(const std::string &_physicsEngine,
                             const std::string &_solverType="quick");
};

/////////////////////////////////////////////////
// copied from speed_thread_pr2.cc
// Get timing information from World::Step()
// \param[in] _world Pointer to the world
// \param[out] _avgTime Average duration of a World::Step
// \param[out] _maxTime Max duration of a World::Step
// \param[out] _minTime Min duration of a World::Step
void stats(physics::WorldPtr _world, common::Time &_avgTime,
    common::Time &_maxTime, common::Time &_minTime)
{
  common::Timer timer;
  common::Time timeLap = common::Time::Zero;

  _avgTime = common::Time::Zero;
  _maxTime = common::Time::Zero;
  _minTime.Set(GZ_INT32_MAX, 0);

  int repetitions = 3;
  int steps = 5000;

  for (int i = 0; i < repetitions; ++i)
  {
    // Time the world for 5000 iterations
    timer.Reset();
    timer.Start();
    _world->Step(steps);
    timer.Stop();

    timeLap = timer.GetElapsed();
    _avgTime += timeLap;
    if (timeLap >= _maxTime)
      _maxTime = timeLap;
    if (timeLap <= _minTime)
      _minTime = timeLap;
  }

  _avgTime = _avgTime.Double() / (repetitions * steps);
}

////////////////////////////////////////////////////////////////////////
void SpeedThreadIslandsTest::RevoluteJoint(const std::string &_physicsEngine,
                                           const std::string &_solverType)
{
  // Load world
  Load("worlds/revolute_joint_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Set solver type and unthrottle update rate
  physics->SetParam("solver_type", _solverType);
  if (_solverType == "world")
  {
    physics->SetParam("ode_quiet", true);
  }
  physics->SetRealTimeUpdateRate(0.0);

  // Expect no island threads by default
  {
    int threads;
    EXPECT_NO_THROW(
      threads = boost::any_cast<int>(physics->GetParam("island_threads")));
    EXPECT_EQ(0, threads);
  }

  // Take 500 steps to warm up.
  world->Step(500);

  // Collect base-line statistics (no threading)
  common::Time baseAvgTime, baseMaxTime, baseMinTime;
  stats(world, baseAvgTime, baseMaxTime, baseMinTime);

  std::cout << "Base Time\n";
  std::cout << "\t Avg[" << baseAvgTime << "]\n"
            << "\t Max[" << baseMaxTime << "]\n"
            << "\t Min[" << baseMinTime << "]\n";

  // Turn on island threads
  {
    int threads;
    physics->SetParam("island_threads", 2);
    EXPECT_NO_THROW(
      threads = boost::any_cast<int>(physics->GetParam("island_threads")));
    EXPECT_EQ(2, threads);
  }

  // Take 500 steps to warm up.
  world->Step(500);

  // Collect threaded statistics
  common::Time threadAvgTime, threadMaxTime, threadMinTime;
  stats(world, threadAvgTime, threadMaxTime, threadMinTime);

  std::cout << "Thread Time\n";
  std::cout << "\t Avg[" << threadAvgTime << "]\n"
            << "\t Max[" << threadMaxTime << "]\n"
            << "\t Min[" << threadMinTime << "]\n";

  // Expect best-case computational time to decrease
  EXPECT_LT(threadMinTime, baseMinTime);
}

TEST_F(SpeedThreadIslandsTest, RevoluteJointQuickStep)
{
  RevoluteJoint("ode", "quick");
}

TEST_F(SpeedThreadIslandsTest, RevoluteJointWorldStep)
{
  RevoluteJoint("ode", "world");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
