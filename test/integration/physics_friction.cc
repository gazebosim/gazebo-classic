/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <cmath>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#ifdef HAVE_BULLET
#include "gazebo/physics/bullet/bullet_math_inc.h"
#endif

#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/gazebo_config.h"

using namespace gazebo;

const double g_friction_tolerance = 1e-3;

class PhysicsFrictionTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  protected: PhysicsFrictionTest() : ServerFixture()
             {
             }

  /// \brief Data structure to hold model pointer and friction parameter
  ///        for each test model in friction demo world.
  class FrictionDemoBox
  {
    public: FrictionDemoBox(physics::WorldPtr _world, const std::string &_name)
            : modelName(_name), world(_world), friction(0.0)
            {
              // Get the model pointer
              model = world->ModelByName(modelName);

              // Get the friction coefficient
              physics::LinkPtr link = model->GetLink();
              physics::Collision_V collisions = link->GetCollisions();
              physics::Collision_V::iterator iter = collisions.begin();
              if (iter != collisions.end())
              {
                physics::SurfaceParamsPtr surf = (*iter)->GetSurface();
                // Use the Secondary friction value,
                // since gravity has a non-zero component in the y direction
                this->friction = surf->FrictionPyramid()->MuSecondary();
              }
            }
    public: ~FrictionDemoBox() {}
    public: std::string modelName;
    public: physics::WorldPtr world;
    public: physics::ModelPtr model;
    public: double friction;
  };

  /// \brief Class to hold parameters for spawning joints.
  public: class SpawnFrictionBoxOptions
  {
    /// \brief Constructor.
    public: SpawnFrictionBoxOptions() : mass(1.0),
              friction1(1.0), friction2(1.0)
            {
            }

    /// \brief Destructor.
    public: ~SpawnFrictionBoxOptions()
            {
            }

    /// \brief Size of box to spawn.
    public: ignition::math::Vector3d size;

    /// \brief Mass of box to spawn (inertia computed automatically).
    public: double mass;

    /// \brief Model pose.
    public: ignition::math::Pose3d modelPose;

    /// \brief Link pose.
    public: ignition::math::Pose3d linkPose;

    /// \brief Inertial pose.
    public: ignition::math::Pose3d inertialPose;

    /// \brief Collision pose.
    public: ignition::math::Pose3d collisionPose;

    /// \brief Friction coefficient in primary direction.
    public: double friction1;

    /// \brief Friction coefficient in secondary direction.
    public: double friction2;

    /// \brief Primary friction direction.
    public: ignition::math::Vector3d direction1;
  };

  /// \brief Spawn a box with friction coefficients and direction.
  /// \param[in] _opt Options for friction box.
  public: physics::ModelPtr SpawnBox(const SpawnFrictionBoxOptions &_opt)
          {
            std::string modelName = this->GetUniqueString("box_model");

            msgs::Model model;
            model.set_name(modelName);
            msgs::Set(model.mutable_pose(), _opt.modelPose);

            msgs::AddBoxLink(model, _opt.mass, _opt.size);
            auto link = model.mutable_link(0);
            msgs::Set(link->mutable_pose(), _opt.linkPose);

            {
              auto inertial = link->mutable_inertial();
              msgs::Set(inertial->mutable_pose(), _opt.inertialPose);
            }

            auto collision = link->mutable_collision(0);
            msgs::Set(collision->mutable_pose(), _opt.collisionPose);

            auto friction = collision->mutable_surface()->mutable_friction();
            friction->set_mu(_opt.friction1);
            friction->set_mu2(_opt.friction2);
            msgs::Set(friction->mutable_fdir1(), _opt.direction1);

            return ServerFixture::SpawnModel(model);
          }

  /// \brief Use the friction_demo world.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void FrictionDemo(const std::string &_physicsEngine,
                            const std::string &_solverType="quick",
                            const std::string &_worldSolverType="ODE_DANTZIG");

  /// \brief Friction test of maximum dissipation principle.
  /// Basically test that friction force vector is aligned with
  /// and opposes velocity vector.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MaximumDissipation(const std::string &_physicsEngine);

  /// \brief Test friction directions for friction pyramid with boxes.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void BoxDirectionRing(const std::string &_physicsEngine);

  /// \brief Use frictionDirection parallel to normal to make sure
  /// no NaN's are generated.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void DirectionNaN(const std::string &_physicsEngine);

  /// \brief Test ode slip parameter on models with 1-3 spheres
  /// and varying mass.
  /// Expect motion to depend on mass, slip, and number of contact points.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void SphereSlip(const std::string &_physicsEngine);
};

class WorldStepFrictionTest : public PhysicsFrictionTest
{
};

/////////////////////////////////////////////////
// FrictionDemo test:
// Uses the test_friction world, which has a bunch of boxes on the ground
// with a gravity vector to simulate a 45-degree inclined plane. Each
// box has a different coefficient of friction. These friction coefficients
// are chosen to be close to the value that would prevent sliding according
// to the Coulomb model.
void PhysicsFrictionTest::FrictionDemo(const std::string &_physicsEngine,
                                       const std::string &_solverType,
                                       const std::string &_worldSolverType)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test since there's an issue with simbody's friction"
          << " parameters (#989)"
          << std::endl;
    return;
  }

  Load("worlds/friction_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  auto g = world->Gravity();

  // Custom gravity vector for this demo world.
  EXPECT_DOUBLE_EQ(g.X(), 0);
  EXPECT_DOUBLE_EQ(g.Y(), -1.0);
  EXPECT_DOUBLE_EQ(g.Z(), -1.0);

  if (_physicsEngine == "ode")
  {
    // Set solver type
    physics->SetParam("solver_type", _solverType);
    if (_solverType == "world")
    {
      physics->SetParam("ode_quiet", true);
    }

    // Set world step solver type
    physics->SetParam("world_step_solver", _worldSolverType);
  }

  std::vector<PhysicsFrictionTest::FrictionDemoBox> boxes;
  std::vector<PhysicsFrictionTest::FrictionDemoBox>::iterator box;
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_01_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_02_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_03_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_04_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_05_model"));
  boxes.push_back(PhysicsFrictionTest::FrictionDemoBox(world, "box_06_model"));

  // Verify box data structure
  for (box = boxes.begin(); box != boxes.end(); ++box)
  {
    ASSERT_TRUE(box->model != NULL);
    ASSERT_GT(box->friction, 0.0);
  }

  common::Time t = world->SimTime();
  while (t.sec < 10)
  {
    world->Step(500);
    t = world->SimTime();

    double yTolerance = g_friction_tolerance;
    if (_solverType == "world")
    {
      if (_worldSolverType == "DART_PGS")
        yTolerance *= 2;
      else if (_worldSolverType == "ODE_DANTZIG")
        yTolerance = 0.84;
    }

    for (box = boxes.begin(); box != boxes.end(); ++box)
    {
      ignition::math::Vector3d vel = box->model->WorldLinearVel();
      EXPECT_NEAR(vel.X(), 0, g_friction_tolerance);
      EXPECT_NEAR(vel.Z(), 0, yTolerance);

      // Coulomb friction model
      if (box->friction >= 1.0)
      {
        // Friction is large enough to prevent motion
        EXPECT_NEAR(vel.Y(), 0, yTolerance);
      }
      else
      {
        // Friction is small enough to allow motion
        // Expect velocity = acceleration * time
        double vyTolerance = yTolerance;
#ifdef HAVE_BULLET
        if (_physicsEngine == "bullet" && sizeof(btScalar) == 4)
        {
          vyTolerance *= 22;
        }
#endif
        EXPECT_NEAR(vel.Y(), (g.Y() + box->friction) * t.Double(),
                    vyTolerance);
      }
    }
  }
  for (box = boxes.begin(); box != boxes.end(); ++box)
  {
    ASSERT_TRUE(box->model != NULL);
  }
}

/////////////////////////////////////////////////
// MaximumDissipation test:
// Start with friction_cone world, which has a circle of boxes,
// set box velocities to different angles,
// expect velocity unit vectors to stay constant while in motion.
void PhysicsFrictionTest::MaximumDissipation(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/friction_cone.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Expect friction cone model
  {
    std::string frictionModel;
    EXPECT_NO_THROW(frictionModel = boost::any_cast<std::string>(
                                      physics->GetParam("friction_model")));
    EXPECT_EQ("cone_model", frictionModel);
  }

  // Get pointers to boxes and their polar coordinate angle
  std::map<physics::ModelPtr, double> modelAngles;

  auto models = world->Models();
  for (auto model : models)
  {
    ASSERT_TRUE(model != nullptr);
    auto name = model->GetName();
    if (0 != name.compare(0, 4, "box_"))
    {
      continue;
    }
    auto pos = model->WorldPose().Pos();
    double angle = std::atan2(pos.Y(), pos.X());
    modelAngles[model] = angle;

    // Expect radius of 9 m
    pos.Z(0);
    double radius = pos.Length();
    EXPECT_NEAR(9.0, radius, 1e-5);

    // Radial velocity should already be set
    auto vel = model->WorldLinearVel();
    EXPECT_GE(vel.Length(), radius*0.95);
    EXPECT_NEAR(angle, atan2(vel.Y(), vel.X()), 1e-6);
  }

  EXPECT_EQ(modelAngles.size(), 32u);

  world->Step(1500);

  gzdbg << "Checking position of boxes" << std::endl;
  std::map<physics::ModelPtr, double>::iterator iter;
  for (iter = modelAngles.begin(); iter != modelAngles.end(); ++iter)
  {
    double angle = iter->second;
    ignition::math::Vector3d pos = iter->first->WorldPose().Pos();
    pos.Z(0);
    double radius = pos.Length();
    double polarAngle = atan2(pos.Y(), pos.X());
    // expect polar angle to remain constant
    EXPECT_NEAR(angle, polarAngle, 1e-2)
      << "model " << iter->first->GetScopedName()
      << std::endl;
    // make sure the boxes are moving outward
    EXPECT_GT(radius, 13)
      << "model " << iter->first->GetScopedName()
      << std::endl;
  }
}

/////////////////////////////////////////////////
// BoxDirectionRing:
// Spawn several boxes with different friction direction parameters.
void PhysicsFrictionTest::BoxDirectionRing(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test since there's an issue with bullet's friction"
          << " parameters (#1045)"
          << std::endl;
    return;
  }
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

  // Load an empty world
  Load("worlds/friction_dir_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set the gravity vector
  ignition::math::Vector3d g(0.0, 1.0, -9.81);
  world->SetGravity(g);

  // Pointers and location of concentric semi-circles of boxes
  std::map<physics::ModelPtr, double> modelAngles;

  auto models = world->Models();
  for (auto model : models)
  {
    ASSERT_TRUE(model != nullptr);
    auto name = model->GetName();
    if (0 != name.compare(0, 4, "box_"))
    {
      continue;
    }
    auto pos = model->WorldPose().Pos();
    double angle = std::atan2(pos.Y(), pos.X());
    modelAngles[model] = angle;
  }
  EXPECT_EQ(modelAngles.size(), 44u);

  // Pointers to spheres model and its links
  physics::ModelPtr spheres = world->ModelByName("spheres");
  ASSERT_TRUE(spheres != nullptr);
  auto sphereLinks = spheres->GetLinks();
  EXPECT_EQ(sphereLinks.size(), 2u);
  for (auto link : sphereLinks)
  {
    ASSERT_TRUE(link != nullptr);
    // spin spheres about vertical axis
    link->SetAngularVel(ignition::math::Vector3d::UnitZ);
  }

  // Step forward
  world->Step(1500);
  double t = world->SimTime().Double();

  gzdbg << "Checking velocity after " << t << " seconds" << std::endl;
  std::map<physics::ModelPtr, double>::iterator iter;
  for (iter = modelAngles.begin(); iter != modelAngles.end(); ++iter)
  {
    double cosAngle = cos(iter->second);
    double sinAngle = sin(iter->second);
    double velMag = g.Y() * sinAngle * t;
    ignition::math::Vector3d vel = iter->first->WorldLinearVel();
    EXPECT_NEAR(velMag*cosAngle, vel.X(), 5*g_friction_tolerance);
    EXPECT_NEAR(velMag*sinAngle, vel.Y(), 5*g_friction_tolerance);
  }
  for (auto link : sphereLinks)
  {
    ASSERT_TRUE(link != nullptr);
    // the friction direction should be in a body-fixed frame
    // so spinning the spheres should cause them to start rolling
    // check that spheres are spinning about the X axis
    auto w = link->WorldAngularVel();
    EXPECT_LT(w.X(), -4) << "Checking " << link->GetScopedName() << std::endl;
  }
}

/////////////////////////////////////////////////
// DirectionNaN:
// Spawn box with vertical friction direction and make sure there's no NaN's
void PhysicsFrictionTest::DirectionNaN(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test since there's an issue with bullet's friction"
          << " parameters (#1045)"
          << std::endl;
    return;
  }
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

  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set the gravity vector
  // small positive y component
  ignition::math::Vector3d g(0.0, 1.5, -1.0);
  world->SetGravity(g);

  // Spawn a single box
  double dx = 0.5;
  double dy = 0.5;
  double dz = 0.2;

  // Set box size and anisotropic friction
  SpawnFrictionBoxOptions opt;
  opt.size.Set(dx, dy, dz);
  opt.direction1 = ignition::math::Vector3d(0.0, 0.0, 1.0);
  opt.modelPose.Pos().Z(dz/2);

  physics::ModelPtr model = SpawnBox(opt);
  ASSERT_TRUE(model != NULL);

  // Step forward
  world->Step(1500);
  double t = world->SimTime().Double();

  gzdbg << "Checking velocity after " << t << " seconds" << std::endl;
  double velMag = (g.Y()+g.Z()) * t;
  ignition::math::Vector3d vel = model->WorldLinearVel();
  EXPECT_NEAR(0.0, vel.X(), g_friction_tolerance);
  EXPECT_NEAR(velMag, vel.Y(), g_friction_tolerance);
}

/////////////////////////////////////////////////
void PhysicsFrictionTest::SphereSlip(const std::string &_physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Aborting test since there's an issue with bullet's friction"
          << " parameters (#1045)"
          << std::endl;
    return;
  }
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

  // Load an empty world
  Load("worlds/friction_spheres.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  // small positive y component
  ignition::math::Vector3d grav = world->Gravity();
  EXPECT_NEAR(grav.X(), 0, 1e-6);
  EXPECT_NEAR(grav.Y(), 2, 1e-6);
  EXPECT_NEAR(grav.Z(), -9.81, 1e-6);

  // Get pointers to models and their mass-slip product
  std::map<physics::ModelPtr, double> lowballMassSlip;
  std::map<physics::ModelPtr, double> twoballMassSlip;
  std::map<physics::ModelPtr, double> triballMassSlip;
  std::map<physics::ModelPtr, double> boxMassSlip;

  auto models = world->Models();
  for (auto model : models)
  {
    ASSERT_TRUE(model != nullptr);
    auto name = model->GetName();
    if (0 != name.compare(3, 5, "ball_"))
    {
      continue;
    }
    double massSlip = std::stod(name.substr(8, 3));
    if (0 == name.compare(0, 3, "low"))
    {
      lowballMassSlip[model] = massSlip;
    }
    else if (0 == name.compare(0, 3, "two"))
    {
      twoballMassSlip[model] = massSlip;
    }
    else if (0 == name.compare(0, 3, "tri"))
    {
      triballMassSlip[model] = massSlip;
    }
    else if (0 == name.compare(0, 3, "box"))
    {
      boxMassSlip[model] = massSlip;
    }
  }

  EXPECT_EQ(lowballMassSlip.size(), 6u);
  EXPECT_EQ(twoballMassSlip.size(), 6u);
  EXPECT_EQ(triballMassSlip.size(), 6u);
  EXPECT_EQ(boxMassSlip.size(), 6u);

  world->Step(5000);

  // With constant lateral gravity and non-zero slip,
  // expect a steady-state lateral velocity that is proportional
  // to the product of slip and mass divided by the number of contact points.
  //
  // The contact points act like viscous dampers in parallel.
  // The slip parameter is defined as:
  //   slip = lateral_velocity / (lateral_force / contact_points)
  // and the velocity is then:
  //   lateral_velocity = lateral_force * slip / contact_points
  //   lateral_velocity = gravity * mass * slip / contact_points
  gzdbg << "Checking velocity of lowball models" << std::endl;
  for (auto lowball : lowballMassSlip)
  {
    auto model = lowball.first;
    double massSlip = lowball.second;
    auto vel = model->WorldLinearVel();
    EXPECT_NEAR(vel.X(), 0, g_friction_tolerance);
    EXPECT_NEAR(vel.Z(), 0, g_friction_tolerance);
    double velExpected = grav.Y() * massSlip / 1.0;
    EXPECT_NEAR(vel.Y(), velExpected, 0.015*velExpected)
      << "model " << lowball.first->GetScopedName()
      << std::endl;
  }
  gzdbg << "Checking velocity of twoball models" << std::endl;
  for (auto twoball : twoballMassSlip)
  {
    auto model = twoball.first;
    double massSlip = twoball.second;
    auto vel = model->WorldLinearVel();
    EXPECT_NEAR(vel.X(), 0, g_friction_tolerance);
    EXPECT_NEAR(vel.Z(), 0, g_friction_tolerance);
    double velExpected = grav.Y() * massSlip / 2.0;
    EXPECT_NEAR(vel.Y(), velExpected, 0.015*velExpected)
      << "model " << twoball.first->GetScopedName()
      << std::endl;
  }
  gzdbg << "Checking velocity of triball models" << std::endl;
  for (auto triball : triballMassSlip)
  {
    auto model = triball.first;
    double massSlip = triball.second;
    auto vel = model->WorldLinearVel();
    EXPECT_NEAR(vel.X(), 0, g_friction_tolerance);
    EXPECT_NEAR(vel.Z(), 0, g_friction_tolerance);
    double velExpected = grav.Y() * massSlip / 3.0;
    EXPECT_NEAR(vel.Y(), velExpected, 0.015*velExpected)
      << "model " << triball.first->GetScopedName()
      << std::endl;
  }
  gzdbg << "Checking velocity of box models" << std::endl;
  for (auto box : boxMassSlip)
  {
    auto model = box.first;
    double massSlip = box.second;
    auto vel = model->WorldLinearVel();
    EXPECT_NEAR(vel.X(), 0, g_friction_tolerance);
    EXPECT_NEAR(vel.Z(), 0, g_friction_tolerance);
    double velExpected = grav.Y() * massSlip / 1.0;
    EXPECT_NEAR(vel.Y(), velExpected, 0.015*velExpected)
      << "model " << box.first->GetScopedName()
      << std::endl;
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, FrictionDemo)
{
  FrictionDemo(GetParam());
}

/////////////////////////////////////////////////
TEST_P(WorldStepFrictionTest, FrictionDemoWorldStep)
{
  std::string worldStepSolver = GetParam();
  if (worldStepSolver.compare("BULLET_PGS") == 0 ||
      worldStepSolver.compare("BULLET_LEMKE") == 0)
  {
    gzerr << "Solver ["
          << worldStepSolver
          << "] doesn't yet work with this test."
          << std::endl;
    return;
  }
  FrictionDemo("ode", "world", worldStepSolver);
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, MaximumDissipation)
{
  if (std::string("ode").compare(GetParam()) == 0)
  {
    MaximumDissipation(GetParam());
  }
  else
  {
    gzerr << "Skipping test for physics engine "
          << GetParam()
          << std::endl;
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, BoxDirectionRing)
{
  BoxDirectionRing(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, DirectionNaN)
{
  DirectionNaN(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsFrictionTest, SphereSlip)
{
  if (std::string("ode").compare(GetParam()) == 0)
  {
    SphereSlip(GetParam());
  }
  else
  {
    gzerr << "Skipping test for physics engine "
          << GetParam()
          << std::endl;
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsFrictionTest,
                        PHYSICS_ENGINE_VALUES);

INSTANTIATE_TEST_CASE_P(WorldStepSolvers, WorldStepFrictionTest,
                        WORLD_STEP_SOLVERS);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
