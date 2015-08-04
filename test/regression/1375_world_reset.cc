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

#include "gazebo/common/Events.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;

bool g_reset = false;

/////////////////////////////////////////////////
void OnReset()
{
  g_reset = true;
}

class Issue1375Test : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue1375Test, WorldReset)
{
  Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  event::ConnectionPtr conn =
    event::Events::ConnectWorldReset(boost::bind(OnReset));

  world->Reset();

  EXPECT_TRUE(g_reset);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
