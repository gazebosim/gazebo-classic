/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>
#include "gazebo/util/Diagnostics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class SpeedThreadPR2Test : public ServerFixture
{
};

/////////////////////////////////////////////////
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

/////////////////////////////////////////////////
// Test ODE threaded position correction.
TEST_F(SpeedThreadPR2Test, PR2SplitImpulseWorld)
{
  Load("worlds/pr2_no_sensors_test.world", true);

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get a pointer to the physics engine
  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  ASSERT_TRUE(physicsEngine != NULL);

  // Unleash the physics engine to maximum speed.
  physicsEngine->SetRealTimeUpdateRate(0);

  // Let things settle
  world->Step(5000);

  // Make sure threading is off
  physicsEngine->SetParam("thread_position_correction", false);

  // Collect base-line statistics (no threading)
  common::Time baseAvgTime, baseMaxTime, baseMinTime;
  stats(world, baseAvgTime, baseMaxTime, baseMinTime);

  std::cout << "Base Time\n";
  std::cout << "\t Avg[" << baseAvgTime << "]\n"
            << "\t Max[" << baseMaxTime << "]\n"
            << "\t Min[" << baseMinTime << "]\n";


  // Turn on threading
  physicsEngine->SetParam("thread_position_correction", true);

  // Collect threaded statistics
  common::Time threadAvgTime, threadMaxTime, threadMinTime;
  stats(world, threadAvgTime, threadMaxTime, threadMinTime);

  std::cout << "Thread Time\n";
  std::cout << "\t Avg[" << threadAvgTime << "]\n"
            << "\t Max[" << threadMaxTime << "]\n"
            << "\t Min[" << threadMinTime << "]\n";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
