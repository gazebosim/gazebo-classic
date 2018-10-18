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
#include "gazebo/physics/ode/ODESurfaceParams.hh"
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
    {
      // Get the model pointer
      this->model = _world->GetModel(_name);

      // Get the link pointer
      this->link = this->model->GetLink();

      // Get surface params
      physics::Collision_V collisions = this->link->GetCollisions();
      auto iter = collisions.begin();
      EXPECT_TRUE(iter != collisions.end());

      auto surf = (*iter)->GetSurface();
      EXPECT_TRUE(surf != NULL);

      auto surfODE =
          boost::static_pointer_cast<physics::ODESurfaceParams>(surf);

      this->coefficient = surf->FrictionPyramid()->MuTorsion();
      this->patch = surf->FrictionPyramid()->PatchRadius();
      this->radius = surf->FrictionPyramid()->SurfaceRadius();
      this->kp = surfODE->kp;

      // Get inertial params
      auto inertial = link->GetInertial();
      EXPECT_TRUE(inertial != NULL);

      this->mass = inertial->GetMass();
      this->izz = inertial->GetIZZ();

      this->error.InsertStatistic("maxAbs");
    }

    /// \brief Pointer to the model
    public: physics::ModelPtr model;

    /// \brief Pointer to the link
    public: physics::LinkPtr link;

    /// \brief Torsional friction
    public: double coefficient;

    /// \brief Spring constant equivalents of a contact.
    public: double kp;

    /// \brief Mass
    public: double mass;

    /// \brief Inertia Izz component
    public: double izz;

    /// \brief Patch
    public: double patch;

    /// \brief Sphere radius
    public: double radius;

    /// \brief Computing signal stats error
    public: ignition::math::SignalStats error;
  };

  /// \brief Use the torsional_friction_test world to test spheres with
  /// different coefficients of torsional friction and different patch radii.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void CoefficientTest(const std::string &_physicsEngine);

  /// \brief Use the torsional_friction_test world to test spheres with
  /// different surface radii.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void RadiusTest(const std::string &_physicsEngine);

  /// \brief Use the torsional_friction_test world to test spheres with
  /// different contact depths. It checks if the contact depth is correct for
  /// those spheres.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void DepthTest(const std::string &_physicsEngine);

  /// \brief Callback for contact subscribers in depth test.
  /// \param[in] _msg Contact message
  private: void Callback(const ConstContactsPtr &_msg);

  /// \brief Message to be filled with the latest contacts message.
  private: msgs::Contacts contactsMsg;

  /// \brief Mutex to protect reads and writes to contactsMsg.
  public: mutable boost::mutex mutex;
};

/////////////////////////////////////////////////
void PhysicsTorsionalFrictionTest::Callback(const ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->contactsMsg = *_msg;
}

/////////////////////////////////////////////////
void PhysicsTorsionalFrictionTest::DepthTest(
    const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Torsional friction only works with ODE (issues #1681 #1682 #1683)"
          << std::endl;
    return;
  }

  Load("test/worlds/torsional_friction_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step so contacts begin
  world->Step(1);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_DOUBLE_EQ(g.x, 0);
  EXPECT_DOUBLE_EQ(g.y, 0);
  EXPECT_DOUBLE_EQ(g.z, -9.8);

  // Sleep to ensure transport topics are all advertised
  common::Time::MSleep(100);
  std::list<std::string> topics =
    transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  topics.sort();
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), 1u);

  auto topic = topics.front();

  gzdbg << "Listening to " << topic << std::endl;
  transport::SubscriberPtr sub = this->node->Subscribe(topic,
      &PhysicsTorsionalFrictionTest::Callback, this);

  // Step for spheres to sink and rest at the final depth
  world->Step(1000);

  // Wait for contact messages to be received
  int maxSleep = 30;
  int sleep = 0;
  while (this->contactsMsg.contact().size() < 20 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  ASSERT_EQ(this->contactsMsg.contact().size(), 20);

  // Copy message to local variable
  msgs::Contacts contacts;
  {
    boost::mutex::scoped_lock lock(this->mutex);
    contacts = this->contactsMsg;
  }

  // Load the spheres
  std::vector<PhysicsTorsionalFrictionTest::SphereData>
      spheres;
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_mass_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_mass_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_mass_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_mass_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_mass_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
  }

  // Check relevant contacts from message
  std::vector<bool> contactsChecked = {false, false, false, false, false};
  ASSERT_EQ(contacts.contact().size(), 20);
  for (auto contact : contacts.contact())
  {
    // Check if the contact has a sphere for this test
    int number = 0;
    if (contact.has_collision1() &&
        contact.collision1().find("sphere_mass") == 0)
    {
      number = std::stoi(contact.collision1().substr(12, 1));
    }
    else if (contact.has_collision2() &&
        contact.collision2().find("sphere_mass") == 0)
    {
      number = std::stoi(contact.collision2().substr(12, 1));
    }
    else
    {
      continue;
    }

    // Get corresponding sphere
    ASSERT_GE(number, 1);
    ASSERT_LE(number, 5);
    contactsChecked[number-1] = true;
    auto sphere = spheres[number-1];

    // Check that contact normal is in the positive Z direction
    ignition::math::Vector3d normal = msgs::ConvertIgn(contact.normal(0));
    EXPECT_EQ(normal, ignition::math::Vector3d::UnitZ);

    // Check that contact depth is:
    // normal force = kp * depth
    double expectedDepth = sphere.mass * -g.z / sphere.kp;
    double relativeError =
      std::abs(contact.depth(0) - expectedDepth)/expectedDepth;
    gzdbg << "sphere_mass_" << number
          << " expected " << expectedDepth
          << " actual " << contact.depth(0)
          << std::endl;

    // Less than 1% error
    EXPECT_LT(relativeError, 0.01);
  }

  // Verify that each sphere was checked
  for (bool contactChecked : contactsChecked)
  {
    EXPECT_TRUE(contactChecked);
  }
}

/////////////////////////////////////////////////
void PhysicsTorsionalFrictionTest::CoefficientTest(
    const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Torsional friction only works with ODE (issues #1681 #1682 #1683)"
          << std::endl;
    return;
  }

  Load("worlds/torsional_friction_test.world", true, _physicsEngine);
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
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_coefficient_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_coefficient_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_coefficient_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_coefficient_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_coefficient_5"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_patch_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_patch_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_patch_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_patch_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_patch_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
    ASSERT_TRUE(sphere.link != NULL);
  }

  // Step so spheres settle in the equilibrium depth
  world->Step(1000);

  int maxSteps = 10;
  int step = 0;
  while (step < maxSteps)
  {
    // Apply a torque about the vertical axis
    double appliedTorque = 1000;
    for (auto sphere : spheres)
    {
      sphere.link->AddRelativeTorque(math::Vector3(0, 0, appliedTorque));
    }

    world->Step(1);
    step++;

    for (auto sphere : spheres)
    {
      // Get angular acceleration
      math::Vector3 acc = sphere.model->GetWorldAngularAccel();
      EXPECT_NEAR(acc.x, 0, g_friction_tolerance);
      EXPECT_NEAR(acc.y, 0, g_friction_tolerance);

      // Calculate torque due to friction
      double normalZ = -sphere.mass * g.z;
      double frictionCoef = sphere.coefficient * 3*M_PI/16 *
          sphere.patch;
      double frictionTorque = normalZ * frictionCoef;
      double frictionAcc = (appliedTorque - frictionTorque) / sphere.izz;
      // Friction is large enough to prevent motion
      if (appliedTorque <= frictionTorque)
      {
        EXPECT_NEAR(acc.z, 0, g_friction_tolerance);
      }
      else
      {
        sphere.error.InsertData(acc.z - frictionAcc);
      }
    }
  }

  // Check error separately to reduce console spam
  for (auto sphere : spheres)
  {
    gzdbg << "Model " << sphere.model->GetName() << std::endl;
    EXPECT_NEAR(sphere.error.Map()["maxAbs"], 0.0, g_friction_tolerance);
  }
}

/////////////////////////////////////////////////
void PhysicsTorsionalFrictionTest::RadiusTest(
    const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Torsional friction only works with ODE (issues #1681 #1682 #1683)"
          << std::endl;
    return;
  }

  Load("worlds/torsional_friction_test.world", true, _physicsEngine);
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
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_radius_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_radius_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_radius_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_radius_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::SphereData(world, "sphere_radius_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
    ASSERT_TRUE(sphere.link != NULL);
  }

  // Step so spheres settle in the equilibrium depth
  world->Step(1000);

  int maxSteps = 10;
  int step = 0;
  while (step < maxSteps)
  {
    // Apply a torque about the vertical axis
    double appliedTorque = 1000;
    for (auto sphere : spheres)
    {
      sphere.link->AddRelativeTorque(math::Vector3(0, 0, appliedTorque));
    }

    world->Step(1);
    step++;
    gzdbg << "world time: " << world->GetSimTime().Double() << std::endl;

    for (auto sphere : spheres)
    {
      // Get angular acceleration
      math::Vector3 acc = sphere.model->GetWorldAngularAccel();
      EXPECT_NEAR(acc.x, 0, g_friction_tolerance);
      EXPECT_NEAR(acc.y, 0, g_friction_tolerance);

      // Calculate torque due to friction
      double depthAtEquilibrium = sphere.mass * -g.z / sphere.kp;
      double patch = sqrt(sphere.radius * depthAtEquilibrium);
      double normalZ = -sphere.mass * g.z;
      double frictionTorque =
          normalZ * sphere.coefficient * 3 * M_PI * patch / 16;
      gzdbg << sphere.model->GetName()
            << " frictionTorque " << frictionTorque
            << std::endl;

      // Friction is large enough to prevent motion
      if (appliedTorque <= frictionTorque)
      {
        EXPECT_NEAR(acc.z, 0, g_friction_tolerance);
      }
      else
      {
        double expectedAcc = (appliedTorque - frictionTorque) / sphere.izz;
        double relativeError = std::abs(acc.z - expectedAcc) / expectedAcc;

        // Less than 1% error
        EXPECT_LT(relativeError, 0.01);
      }
    }
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsTorsionalFrictionTest, CoefficientTest)
{
  CoefficientTest(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsTorsionalFrictionTest, RadiusTest)
{
  RadiusTest(GetParam());
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
