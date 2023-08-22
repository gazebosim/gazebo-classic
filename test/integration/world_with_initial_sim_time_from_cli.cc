/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include <string>
#include "gazebo/common/Time.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

const double initialSimTime = 1675117690.123456;

/// \brief Helper class that initializes each test.
class WorldWithInitialSimTimeFromCliTest : public ServerFixture
{
  /// \brief Class constructor.
  public: WorldWithInitialSimTimeFromCliTest()
  {
    // Start the server with an inital sim time and paused.
    this->LoadArgs("-u --initial_sim_time " + std::to_string(initialSimTime));
    this->world = physics::get_world("default");
  }

  /// \brief Pointer to the world.
  public: physics::WorldPtr world;
};

/////////////////////////////////////////////////
/// \brief Check that the initial simulation time is set correctly.
TEST_F(WorldWithInitialSimTimeFromCliTest, CheckInitialSimTime)
{
  ASSERT_TRUE(this->world != NULL);
  EXPECT_TRUE(this->world->IsPaused());

  // check that the simulation time is the same as the initial sim time
  EXPECT_DOUBLE_EQ(this->world->SimTime().Double(), initialSimTime);

  // check that after a step, the simulation time advances
  this->world->Step(2);
  EXPECT_GT(this->world->SimTime().Double() - initialSimTime, 1e-4);

  // check that the simulation time is again the same as the initial sim time
  // after a reset
  this->world->Reset();
  EXPECT_DOUBLE_EQ(this->world->SimTime().Double(), initialSimTime);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
