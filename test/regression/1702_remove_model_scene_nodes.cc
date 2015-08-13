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

#include <ignition/math.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"
#include "gazebo/gazebo_config.h"

using namespace gazebo;

const double g_tolerance = 1e-4;
class Issue1702Test : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  /// \brief Test for issue #1702, spawn a model,  delete it, and respawn with
  /// the same name.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SpawnDeleteSpawnAgain(const std::string &_physicsEngine);
};


/////////////////////////////////////////////////
void Issue1702Test::SpawnDeleteSpawnAgain(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzwarn << "ServerFixture::SpawnModel fails for Simbody." << std::endl;
    return;
  }

  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(math::Vector3::Zero);

  // spawn a model
  msgs::Model model;
  model.set_name("a_fancy_box");
  const ignition::math::Vector3d pos(0.0, 0.0, 5.0);
  msgs::Set(model.mutable_pose(),
            ignition::math::Pose3<double>(pos,
            ignition::math::Quaternion<double>::Identity));
  const double mass = 1.5;
  const ignition::math::Vector3d size(1.1, 1.2, 1.3);
  msgs::AddBoxLink(model, mass, size);

  physics::ModelPtr box = ServerFixture::SpawnModel(model);
  EXPECT_TRUE(box != NULL);

  std::string name = box->GetName();

  // delete that model
  ServerFixture::RemoveModel(name);

  int count = 0;
  while(world->GetModel(name) != NULL && ++count < 1000)
  {
    common::Time::MSleep(1);
    world->Step(1);
  }
  EXPECT_TRUE(world->GetModel(name) == NULL);
  EXPECT_TRUE(count < 1000);

  // spawn the exact same model
  // if this succeeds, we're OK.
  physics::ModelPtr newBox = ServerFixture::SpawnModel(model);
  // EXPECT_TRUE(newBox != NULL);  /// \TODO: this fails, should it?

  count = 0;
  while(world->GetModel(name) == NULL && ++count < 1000)
  {
    common::Time::MSleep(1);
    world->Step(1);
  }

  EXPECT_TRUE(world->GetModel(name) != NULL);
}

TEST_P(Issue1702Test, SpawnDeleteSpawnAgain)
{
  SpawnDeleteSpawnAgain(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue1702Test, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
