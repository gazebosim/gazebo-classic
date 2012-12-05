/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "physics/physics.hh"

using namespace gazebo;
class BulletTest : public ServerFixture
{
};


TEST_F(BulletTest, EmptyWorldTest)
{
  // check conservation of mementum for linear inelastic collision
  Load("worlds/empty_bullet.world", true);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // simulate a couple seconds
  world->StepWorld(2000);
  double t = world->GetSimTime().Double();
  EXPECT_TRUE(t > 0);

  Unload();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
