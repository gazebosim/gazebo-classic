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
#include <string.h>
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class NonDefaultWorld : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void Load(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void NonDefaultWorld::Load(const std::string &_physicsEngine)
{
  ServerFixture::Load("worlds/empty_different_name.world", false,
    _physicsEngine);
  physics::WorldPtr world = physics::get_world("not_the_default_world_name");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("ground_plane");
  ASSERT_TRUE(model != NULL);
}

TEST_P(NonDefaultWorld, Load)
{
  Load(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, NonDefaultWorld,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
