/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <gazebo/rendering/rendering.hh>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue351Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test for issue #351
TEST_F(Issue351Test, WorldStep)
{
  Load("worlds/world_step.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Take 500 steps; it passes if it doesn't seg-fault
  world->Step(500);
}
