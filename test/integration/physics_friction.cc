/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsFrictionTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  class FrictionBox
  {
    public: FrictionBox(physics::WorldPtr _world, const std::string &_name)
            : modelName(_name), world(_world), friction(0.0)
            {
              model = world->GetModel(modelName);
            }
    public: ~FrictionBox() {}
    public: std::string modelName;
    public: physics::WorldPtr world;
    public: physics::ModelPtr model;
    public: double friction;
  };
  public: void FrictionDemo(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsFrictionTest::FrictionDemo(const std::string &_physicsEngine)
{
  Load("worlds/test_friction.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  // Custom gravity vector for this demo world.
  EXPECT_DOUBLE_EQ(g.x, 0);
  EXPECT_DOUBLE_EQ(g.y, -1.0);
  EXPECT_DOUBLE_EQ(g.z, -1.0);

  std::vector<PhysicsFrictionTest::FrictionBox> boxes;
  std::vector<PhysicsFrictionTest::FrictionBox>::iterator box;
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_01_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_02_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_03_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_04_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_05_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionBox(world, "box_06_model"));

  for (box = boxes.begin(); box != boxes.end(); ++box)
  {
    ASSERT_TRUE(box->model != NULL);
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, FrictionDemo)
{
  FrictionDemo(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsFrictionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
