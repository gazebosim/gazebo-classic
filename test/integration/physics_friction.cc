/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

using namespace gazebo;

const double g_friction_tolerance = 1e-3;

class PhysicsFrictionTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Data structure to hold model pointer and friction parameter
  ///        for each test model.
  class FrictionBox
  {
    public: FrictionBox(physics::WorldPtr _world, const std::string &_name)
            : modelName(_name), world(_world), friction(0.0)
            {
              // Get the model pointer
              model = world->GetModel(modelName);

              // Get the friction coefficient
              physics::LinkPtr link = model->GetLink();
              physics::Collision_V collisions = link->GetCollisions();
              physics::Collision_V::iterator iter = collisions.begin();
              if (iter != collisions.end())
              {
                physics::SurfaceParamsPtr surface = (*iter)->GetSurface();
                // Average the mu1 and mu2 values
                this->friction = (surface->mu1 + surface->mu2) / 2.0;
              }
            }
    public: ~FrictionBox() {}
    public: std::string modelName;
    public: physics::WorldPtr world;
    public: physics::ModelPtr model;
    public: double friction;
  };
  public: void ColoumbFriction(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
// ColoumbFriction test:
// Uses the test_friction world, which has a bunch of boxes on the ground
// with a gravity vector to simulate a 45-degree inclined plane. Each
// box has a different coefficient of friction. These friction coefficients
// are chosen to be close to the value that would prevent sliding according
// to the Coloumb model.
void PhysicsFrictionTest::ColoumbFriction(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test since there's an issue with simbody's friction"
          << " parameters (#989)"
          << std::endl;
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test since there's an issue with dart's friction"
          << " parameters (#1000)"
          << std::endl;
    return;
  }

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

  // Verify box data structure
  for (box = boxes.begin(); box != boxes.end(); ++box)
  {
    ASSERT_TRUE(box->model != NULL);
    ASSERT_GT(box->friction, 0.0);
  }

  common::Time t = world->GetSimTime();
  while (t.sec < 10)
  {
    world->StepWorld(500);
    t = world->GetSimTime();

    for (box = boxes.begin(); box != boxes.end(); ++box)
    {
      math::Vector3 vel = box->model->GetWorldLinearVel();
      EXPECT_NEAR(vel.x, 0, g_friction_tolerance);
      EXPECT_NEAR(vel.z, 0, g_friction_tolerance);

      // Coulomb friction model
      if (box->friction >= 1.0)
      {
        // Friction is large enough to prevent motion
        EXPECT_NEAR(vel.y, 0, g_friction_tolerance);
      }
      else
      {
        // Friction is small enough to allow motion
        // Expect velocity = acceleration * time
        EXPECT_NEAR(vel.y, (g.y + box->friction) * t.Double(),
                    g_friction_tolerance);
      }
    }
  }
  for (box = boxes.begin(); box != boxes.end(); ++box)
  {
    ASSERT_TRUE(box->model != NULL);
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, ColoumbFriction)
{
  ColoumbFriction(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsFrictionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
