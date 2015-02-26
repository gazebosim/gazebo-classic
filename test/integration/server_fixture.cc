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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/physics.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class ServerFixtureTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
  public: void LoadPaused(const std::string &_physicsType);
  public: void LoadEmptyOfType(const std::string &_physicsType);
  public: void SpawnSDF(const std::string &_physicsType);
};

////////////////////////////////////////////////////////////////////////
// LoadPaused:
// Verify that ServerFixture can load world in paused state
// Gazebo issue #334
////////////////////////////////////////////////////////////////////////
void ServerFixtureTest::LoadPaused(const std::string &_physicsType)
{
  // Note the second argument of Load sets the pause state
  Load("worlds/empty.world", true, _physicsType);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  gzdbg << "Check IsPaused with no delay\n";
  EXPECT_TRUE(world->IsPaused());

  common::Time::MSleep(100);
  gzdbg << "Check IsPaused with 100 ms delay\n";
  EXPECT_TRUE(world->IsPaused());

  common::Time::MSleep(900);
  gzdbg << "Check IsPaused with 1000 ms delay\n";
  EXPECT_TRUE(world->IsPaused());
}

TEST_P(ServerFixtureTest, LoadPaused)
{
  LoadPaused(GetParam());
}

////////////////////////////////////////////////////////////////////////
// LoadEmptyOfType:
// Verify that ServerFixture can load empty world with different types
// of physics engines (issue #486)
////////////////////////////////////////////////////////////////////////
void ServerFixtureTest::LoadEmptyOfType(const std::string &_physicsType)
{
  // Note the second argument of Load sets the pause state
  Load("worlds/empty.world", true, _physicsType);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsType);
}

TEST_P(ServerFixtureTest, LoadEmptyOfType)
{
  LoadEmptyOfType(GetParam());
}

////////////////////////////////////////////////////////////////////////
// SpawnSDF:
// Verify that the SpawnSDF function does not get stuck in a loop
// Gazebo issue #530
////////////////////////////////////////////////////////////////////////
void ServerFixtureTest::SpawnSDF(const std::string &_physicsType)
{
  // Note the second argument of Load sets the pause state
  Load("worlds/blank.world", true, _physicsType);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  EXPECT_TRUE(world->IsPaused());

  std::stringstream sdfStr;
  math::Pose pose(1, 2, 3, 0, 0, 0);
  sdfStr << "<sdf version='" << SDF_VERSION << "'>"
         << "<model name='box'>"
         << "  <pose>" << pose << "</pose>"
         << "  <link name='link'>"
         << "    <collision name='col'>"
         << "      <geometry>"
         << "        <box><size>1 1 1</size></box>"
         << "      </geometry>"
         << "    </collision>"
         << "    <visual name='vis'>"
         << "      <geometry>"
         << "        <box><size>1 1 1</size></box>"
         << "      </geometry>"
         << "    </visual>"
         << "  </link>"
         << "</model>"
         << "</sdf>";
  ServerFixture::SpawnSDF(sdfStr.str());

  physics::ModelPtr model;
  model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);

  EXPECT_EQ(pose.pos, model->GetWorldPose().pos);
}

TEST_P(ServerFixtureTest, SpawnSDF)
{
  SpawnSDF(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ServerFixtureTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
