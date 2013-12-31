/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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
#include "gazebo/physics/Joint_TEST.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

class Issue978Test : public Joint_TEST
{
  public: void CollisionWorldPose(const std::string &_physicsEngine);
};


/////////////////////////////////////////////////
// \brief Test for issue #asdf
void Issue978Test::CollisionWorldPose(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // add a link with a collision body in it

  // Create a new link
  SpawnBox("box_1", math::Vector3(1, 1, 1), math::Vector3(0, 0, 1),
    math::Vector3(0.5, 0.5, 0.5));

  physics::ModelPtr model = world->GetModel("box_1");

  physics::Link_V links = model->GetLinks();
  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    physics::LinkPtr link = *li;
    physics::Collision_V collisions = link->GetCollisions();
    for (physics::Collision_V::iterator ci = collisions.begin();
       ci != collisions.end(); ++ci)
    {
      gzdbg << "abs pose [" << (*ci)->GetWorldPose()
            << "] rel pose [" << (*ci)->GetRelativePose() << "]\n";
      EXPECT_EQ((*ci)->GetWorldPose().pos, math::Vector3(0, 0, 1));
      EXPECT_EQ((*ci)->GetWorldPose().rot.GetAsEuler(),
        math::Vector3(0.5, 0.5, 0.5));
    }
  }
}

TEST_P(Issue978Test, CollisionWorldPose)
{
  CollisionWorldPose(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue978Test,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("")));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
