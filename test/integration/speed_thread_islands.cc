/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

class SpeedThreadIslandsTest : public ServerFixture,
                               public testing::WithParamInterface<const char*>
{
  /// \brief Load a world file and test the thread speedup using _threads
  /// Unthrottle update rate, set island threads, and check
  /// changes in required computational time.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _solverType Type of solver to use.
  /// \param[in] _worldFile The world file to load into physics engine.
  /// \param[in] _threads The number of threads to use for speedup test.
  /// \param[in] _warmUpSteps The numebr of warm up simulation steps.
  public: void ThreadSpeedup(const std::string &_physicsEngine,
                             const std::string &_solverType,
                             const std::string &_worldFile,
                             const int _threads,
                             const int _warmUpSteps);
};

/////////////////////////////////////////////////
// copied from speed_thread_pr2.cc
// Get timing information from World::Step()
// \param[in] _world Pointer to the world
// \param[in] _steps Number of steps to run
// \param[out] _avgTime Average duration of a World::Step
// \param[out] _maxTime Max duration of a World::Step
// \param[out] _minTime Min duration of a World::Step
void Stats(physics::WorldPtr _world, const int _steps, common::Time &_avgTime,
    common::Time &_maxTime, common::Time &_minTime)
{
  common::Timer timer;
  common::Time timeLap = common::Time::Zero;

  _avgTime = common::Time::Zero;
  _maxTime = common::Time::Zero;
  _minTime.Set(ignition::math::MAX_I32, 0);

  int repetitions = 3;
  int steps = _steps;

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

  _avgTime = _avgTime.Double() / repetitions;
}

////////////////////////////////////////////////////////////////////////
void SpeedThreadIslandsTest::ThreadSpeedup(const std::string &_physicsEngine,
                                           const std::string &_solverType,
                                           const std::string &_worldFile,
                                           const int _threads,
                                           const int _warmUpSteps)
{
  // Load world
  Load(_worldFile, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
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

  // Take some steps to warm up.
  world->Step(_warmUpSteps);

  // Collect base-line statistics (no threading)
  common::Time baseAvgTime, baseMaxTime, baseMinTime;
  Stats(world, 10*_warmUpSteps, baseAvgTime, baseMaxTime, baseMinTime);

  std::cout << "Base Time\n";
  std::cout << "\t Avg[" << baseAvgTime << "]\n"
            << "\t Max[" << baseMaxTime << "]\n"
            << "\t Min[" << baseMinTime << "]\n";

  // Turn on island threads
  {
    int threads;
    physics->SetParam("island_threads", _threads);
    EXPECT_NO_THROW(
      threads = boost::any_cast<int>(physics->GetParam("island_threads")));
    EXPECT_EQ(_threads, threads);
  }

  // Take some steps to warm up.
  world->Step(_warmUpSteps);

  // Collect threaded statistics
  common::Time threadAvgTime, threadMaxTime, threadMinTime;
  Stats(world, 10*_warmUpSteps, threadAvgTime, threadMaxTime, threadMinTime);

  std::cout << "Thread Time\n";
  std::cout << "\t Avg[" << threadAvgTime << "]\n"
            << "\t Max[" << threadMaxTime << "]\n"
            << "\t Min[" << threadMinTime << "]\n";

  // Expect best-case computational time to decrease
  EXPECT_LT(threadMinTime, baseMinTime);
}


TEST_F(SpeedThreadIslandsTest, MultiplePendulumQuickStep)
{
  ThreadSpeedup("ode", "quick",
    "worlds/revolute_joint_test_with_large_gap.world", 4, 500);
}

// this test fails on macOS, see issue #2364
#ifndef __APPLE__
TEST_F(SpeedThreadIslandsTest, MultiplePendulumWorldStep)
{
  ThreadSpeedup("ode", "world",
    "worlds/revolute_joint_test_with_large_gap.world", 4, 500);
}
#endif

TEST_F(SpeedThreadIslandsTest, DualPR2QuickStep)
{
  ThreadSpeedup("ode", "quick", "worlds/dual_pr2.world", 2, 50);
}

TEST_F(SpeedThreadIslandsTest, DualPR2WorldStep)
{
  ThreadSpeedup("ode", "world", "worlds/dual_pr2.world", 2, 50);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
