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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

const double g_friction_tolerance = 1e-3;

class PhysicsTorsionalFrictionTest : public ServerFixture,
    public testing::WithParamInterface<const char*>
{
  protected: PhysicsTorsionalFrictionTest() : ServerFixture()
  {
  }

  /// \brief Data structure to hold model pointer and friction parameter
  /// for each test model in friction demo world.
  class SphereData
  {
    /// \brief Constructor.
    /// \param[in] _world World pointer.
    /// \param[in] _name Model name.
    public: SphereData(physics::WorldPtr _world,
        const std::string &_name)
        : torsionalFriction(0.0)
    {
      // Get the model pointer
      this->model = _world->GetModel(_name);

      // Get the friction coefficient
      physics::LinkPtr link = this->model->GetLink();
      physics::Collision_V collisions = link->GetCollisions();
      auto iter = collisions.begin();
      if (iter != collisions.end())
      {
        physics::SurfaceParamsPtr surf = (*iter)->GetSurface();
        this->torsionalFriction = surf->GetFrictionPyramid()->GetMuTorsion();
      }
    }

    /// \brief Destructor
    public: ~SphereData() = default;

    /// \brief Pointer to the model
    public: physics::ModelPtr model;

    /// \brief Torsional friction
    public: double torsionalFriction;
  };

  /// \brief Use the torsional_friction_demo world to test spheres with
  /// different coefficients of friction.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void CoefficientTest(const std::string &_physicsEngine);

  /// \brief Use the torsional_friction_demo world to test spheres with
  /// different contact depths.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void DepthTest(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
// Coefficient test:
// Uses the torsional_friction_demo world, which has several models rotating.
// Some of these models are spheres rotating about the Z axis, each one with a
// different coefficient of torsional friction.
void PhysicsTorsionalFrictionTest::CoefficientTest(
    const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Torsional friction only works with ODE (#ISSUE)"
          << std::endl;
    return;
  }

  Load("worlds/torsional_friction_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_DOUBLE_EQ(g.x, 0);
  EXPECT_DOUBLE_EQ(g.y, 0);
  EXPECT_DOUBLE_EQ(g.z, -9.8);

  // Load the spheres
  std::vector<PhysicsTorsionalFrictionTest::SphereData>
      spheres;
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
  }

  common::Time t = world->GetSimTime();
  while (t.sec < 1)
  {
    world->Step(500);
    t = world->GetSimTime();

    for (auto sphere : spheres)
    {
      math::Vector3 vel = sphere.model->GetWorldAngularVel();
      EXPECT_NEAR(vel.x, 0, g_friction_tolerance);
      EXPECT_NEAR(vel.y, 0, g_friction_tolerance);

      // Coulomb friction model
      if (sphere.torsionalFriction >= 1.0)
      {
        // Friction is large enough to prevent motion
        EXPECT_NEAR(vel.z, 0, g_friction_tolerance);
      }
      else
      {
        // Friction is small enough to allow motion
        // Expect velocity = acceleration * time
//        EXPECT_NEAR(vel.z, (g.z + sphere.torsionalFriction) * t.Double(),
  //                  g_friction_tolerance);
      }
    }
  }
}

/////////////////////////////////////////////////
// Depth test:
// Uses the torsional_friction_demo world, which has several spheres. It checks
// if the contact depth is correct for those spheres.
void PhysicsTorsionalFrictionTest::DepthTest(
    const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Torsional friction only works with ODE (#ISSUE)"
          << std::endl;
    return;
  }

  Load("worlds/torsional_friction_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_DOUBLE_EQ(g.x, 0);
  EXPECT_DOUBLE_EQ(g.y, 0);
  EXPECT_DOUBLE_EQ(g.z, -9.8);

  // Get contact manager
  physics::ContactManager *mgr = physics->GetContactManager();
  ASSERT_TRUE(mgr != NULL);

  // Load the spheres
  std::vector<PhysicsTorsionalFrictionTest::SphereData>
      spheres;
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
  }

  // Get contacts
  std::vector<physics::Contact *> contacts;

  int maxTries = 100;
  while (contacts.size() == 0 && maxTries > 0)
  {
    world->Step(1);
    contacts = mgr->GetContacts();

    std::cout << "Contacts: " << contacts.size()  << std::endl;
    maxTries--;
  }

  // Check depth
  for (auto sphere : spheres)
  {
    bool contactFound = false;
    for (auto contact : contacts)
    {
      if (contact->collision1->GetLink()->GetModel() != sphere.model)
        continue;
      contactFound = true;

      std::cout << "Depth: " << contact->depths[0] << std::endl;
    }
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsTorsionalFrictionTest, CoefficientTest)
{
  CoefficientTest(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsTorsionalFrictionTest, DepthTest)
{
  DepthTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTorsionalFrictionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
