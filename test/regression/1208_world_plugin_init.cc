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

#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

class Issue1208Test : public ServerFixture
{
};


/////////////////////////////////////////////////
// \brief Test for issue #1208
TEST_F(Issue1208Test, Reset)
{
  Load("worlds/issue_1208.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  // There is a WorldPlugin attached to this world.
  // It has a counter that increments with each call of Init.
  // It then sets the value of that counter to the
  // real time update rate of the physics engine.
  // It's a little convoluted, but it's an easy way to get
  // data from the plugin without using transport.
  EXPECT_NEAR(physics->GetRealTimeUpdateRate(), 1.0, 1e-2);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
