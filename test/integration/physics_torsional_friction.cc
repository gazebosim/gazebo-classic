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
  protected: PhysicsTorsionalFrictionTest() : ServerFixture(), spawnCount(0)
  {
  }

  /// \brief Data structure to hold model pointer and friction parameter
  ///        for each test model in friction demo world.
  class TorsionalFrictionDemoSphere
  {
    /// \brief Constructor.
    /// \param[in] _world World pointer.
    /// \param[in] _name Model name.
    public: TorsionalFrictionDemoSphere(physics::WorldPtr _world,
        const std::string &_name)
        : modelName(_name), world(_world), torsionalFriction(0.0)
    {
      // Get the model pointer
      this->model = this->world->GetModel(this->modelName);

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
    public: ~TorsionalFrictionDemoSphere() {}

    /// \brief Model name
    public: std::string modelName;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the model
    public: physics::ModelPtr model;

    /// \brief Torsional friction
    public: double torsionalFriction;
  };

  /// \brief Class to hold parameters for spawning spheres.
  public: class SpawnTorsionalFrictionSphereOptions
  {
    /// \brief Constructor.
    public: SpawnTorsionalFrictionSphereOptions() : mass(1.0),
        torsionalFriction(1.0)
    {
    }

    /// \brief Destructor.
    public: ~SpawnTorsionalFrictionSphereOptions()
    {
    }

    /// \brief Radius of sphere to spawn.
    public: double radius;

    /// \brief Mass of sphere to spawn (inertia computed automatically).
    public: double mass;

    /// \brief Model pose.
    public: math::Pose modelPose;

    /// \brief Link pose.
    public: math::Pose linkPose;

    /// \brief Inertial pose.
    public: math::Pose inertialPose;

    /// \brief Collision pose.
    public: math::Pose collisionPose;

    /// \brief Torsional friction.
    public: double torsionalFriction;
  };

  /// \brief Spawn a sphere with friction coefficients
  /// \param[in] _opt Options for friction sphere.
  public: physics::ModelPtr SpawnSphere(
      const SpawnTorsionalFrictionSphereOptions &_opt)
  {
    msgs::Factory msg;
    std::ostringstream modelStr;
    std::ostringstream modelName;
    modelName << "sphere_model" << this->spawnCount++;

    double r = _opt.radius;
    double m = _opt.mass;
    double ixx = (2 * m * r * r) / 5;
    double iyy = (2 * m * r * r) / 5;
    double izz = (2 * m * r * r) / 5;

    modelStr
      << "<sdf version='" << SDF_VERSION << "'>"
      << "<model name ='" << modelName.str() << "'>"
      << "  <pose>" << _opt.modelPose << "</pose>"
      << "  <link name='link'>"
      << "    <pose>" << _opt.linkPose << "</pose>"
      << "    <inertial>"
      << "      <pose>" << _opt.inertialPose << "</pose>"
      << "      <mass>" << _opt.mass << "</mass>"
      << "      <inertia>"
      << "        <ixx>" << ixx << "</ixx>"
      << "        <iyy>" << iyy << "</iyy>"
      << "        <izz>" << izz << "</izz>"
      << "        <ixy>" << 0.0 << "</ixy>"
      << "        <ixz>" << 0.0 << "</ixz>"
      << "        <iyz>" << 0.0 << "</iyz>"
      << "      </inertia>"
      << "    </inertial>"
      << "    <collision name='collision'>"
      << "      <pose>" << _opt.collisionPose << "</pose>"
      << "      <geometry>"
      << "        <sphere><radius>" << _opt.radius << "</radius></sphere>"
      << "      </geometry>"
      << "      <surface>"
      << "        <friction>"
      << "          <ode>"
      << "            <mu3>" << _opt.torsionalFriction << "</mu3>"
      << "          </ode>"
      << "        </friction>"
      << "      </surface>"
      << "    </collision>"
      << "    <visual name='visual'>"
      << "      <pose>" << _opt.collisionPose << "</pose>"
      << "      <geometry>"
      << "        <sphere><radius>" << _opt.radius << "</radius></sphere>"
      << "      </geometry>"
      << "    </visual>"
      << "  </link>"
      << "</model>";

    physics::WorldPtr world = physics::get_world("default");
    world->InsertModelString(modelStr.str());

    physics::ModelPtr model;
    common::Time wait(100, 0);

    common::Time wallStart = common::Time::GetWallTime();
    unsigned int waitCount = 0;
    while (wait > (common::Time::GetWallTime() - wallStart) &&
        !this->HasEntity(modelName.str()))
    {
      common::Time::MSleep(10);
      if (++waitCount % 100 == 0)
      {
        gzwarn << "Waiting " << waitCount / 100 << " seconds for "
               << "sphere to spawn." << std::endl;
      }
    }
    if (this->HasEntity(modelName.str()) && waitCount >= 100)
      gzwarn << "sphere has spawned." << std::endl;

    if (world != NULL)
      model = world->GetModel(modelName.str());

    return model;
  }

  /// \brief Use the torsional_friction world.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void VelocityTest(const std::string &_physicsEngine);

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

/////////////////////////////////////////////////
// TorsionalFrictionDemo test:
// Uses the torsional_friction world, which has a bunch of spheres on the ground
// with a gravity vector to simulate a 45-degree inclined plane. Each
// sphere has a different coefficient of friction. These friction coefficients
// are chosen to be close to the value that would prevent sliding according
// to the Coulomb model.
void PhysicsTorsionalFrictionTest::VelocityTest(
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
  std::vector<PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere>
      spheres;
  spheres.push_back(
      PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere(world,
      "sphere_1"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere(world,
      "sphere_2"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere(world,
      "sphere_3"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere(world,
      "sphere_4"));
  spheres.push_back(
      PhysicsTorsionalFrictionTest::TorsionalFrictionDemoSphere(world,
      "sphere_5"));

  // Verify sphere data structure
  for (auto sphere : spheres)
  {
    ASSERT_TRUE(sphere.model != NULL);
  }

  common::Time t = world->GetSimTime();
  while (t.sec < 10)
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
TEST_P(PhysicsTorsionalFrictionTest, TorsionalFrictionDemo)
{
  VelocityTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTorsionalFrictionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
