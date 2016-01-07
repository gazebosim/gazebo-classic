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
              model = world->GetModel(modelName);

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
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  // Custom gravity vector for this demo world.
  EXPECT_DOUBLE_EQ(g.x, 0);
  EXPECT_DOUBLE_EQ(g.y, -1.0);
  EXPECT_DOUBLE_EQ(g.z, -1.0);

  if (_physicsEngine == "ode")
  {
    // Set solver type
    physics->SetParam("solver_type", _solverType);

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

  common::Time t = world->GetSimTime();
  while (t.sec < 10)
  {
    world->Step(500);
    t = world->GetSimTime();

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
      math::Vector3 vel = box->model->GetWorldLinearVel();
      EXPECT_NEAR(vel.x, 0, g_friction_tolerance);
      EXPECT_NEAR(vel.z, 0, yTolerance);

      // Coulomb friction model
      if (box->friction >= 1.0)
      {
        // Friction is large enough to prevent motion
        EXPECT_NEAR(vel.y, 0, yTolerance);
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
        EXPECT_NEAR(vel.y, (g.y + box->friction) * t.Double(),
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
// Start with empty world,
// spawn a bunch of boxes,
// sets box velocities to different angles,
// expect velocity unit vectors to stay constant while in motion.
void PhysicsFrictionTest::MaximumDissipation(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // get the gravity vector
  // small positive y component
  math::Vector3 g = physics->GetGravity();

  // Set friction model
  // "cone_model", "pyramid_model", "box_model"
  const std::string frictionModel = "cone_model";
  physics->SetParam("friction_model", frictionModel);

  // Spawn concentric semi-circles of boxes
  int boxes = 32;
  double dx = 0.5;
  double dy = 0.5;
  double dz = 0.2;
  std::map<physics::ModelPtr, double> modelAngles;

  for (int ring = 0; ring < 5; ++ring)
  {
    gzdbg << "Spawn ring " << ring+1 << " of boxes" << std::endl;
    for (int i = 0; i < boxes; ++i)
    {
      // Set box size and anisotropic friction
      SpawnFrictionBoxOptions opt;
      opt.size.Set(dx, dy, dz);
      opt.friction1 = 0.3;
      opt.friction2 = opt.friction1;

      // Compute angle for each box
      double radius = 9.0 + ring;
      double angle = 2*M_PI*static_cast<double>(i) / static_cast<double>(boxes);
      opt.modelPose.Pos().Set(radius*cos(angle), radius*sin(angle), dz/2);

      if (ring == 0)
        opt.direction1 = ignition::math::Vector3d(-sin(angle), cos(angle), 0);
      else if (ring < 4)
        opt.direction1 = ignition::math::Vector3d(0.0, 1.0, 0.0);

      if (ring == 1)
        opt.collisionPose.Rot().Euler(0.0, 0.0, angle);

      if (ring == 2)
        opt.linkPose.Rot().Euler(0.0, 0.0, angle);

      if (ring == 3)
        opt.modelPose.Rot().Euler(0.0, 0.0, angle);

      physics::ModelPtr model = SpawnBox(opt);
      ASSERT_TRUE(model != NULL);
      modelAngles[model] = angle;

      // Set velocity, larger for outer rings.
      model->SetLinearVel(
          radius * ignition::math::Vector3d(cos(angle), sin(angle), 0));
    }
  }

  world->Step(1500);

  gzdbg << "Checking position of boxes" << std::endl;
  std::map<physics::ModelPtr, double>::iterator iter;
  for (iter = modelAngles.begin(); iter != modelAngles.end(); ++iter)
  {
    double cosAngle = cos(iter->second);
    double sinAngle = sin(iter->second);
    math::Vector3 pos = iter->first->GetWorldPose().pos;
    double cosPosAngle = pos.x / pos.GetLength();
    double sinPosAngle = pos.y / pos.GetLength();
    EXPECT_NEAR(cosAngle, cosPosAngle, 1e-2);
    EXPECT_NEAR(sinAngle, sinPosAngle, 1e-2);
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
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set the gravity vector
  math::Vector3 g(0.0, 1.0, -9.81);
  physics->SetGravity(g);

  // Spawn concentric semi-circles of boxes
  int boxes = 10;
  double dx = 0.5;
  double dy = 0.5;
  double dz = 0.2;
  std::map<physics::ModelPtr, double> modelAngles;

  for (int ring = 0; ring < 4; ++ring)
  {
    gzdbg << "Spawn ring " << ring+1 << " of boxes" << std::endl;
    for (int i = 0; i <= boxes; ++i)
    {
      // Set box size and anisotropic friction
      SpawnFrictionBoxOptions opt;
      opt.size.Set(dx, dy, dz);
      opt.friction1 = 100.0;
      opt.friction2 = 0.0;

      // Compute angle for each box
      double radius = 5.0 + ring;
      double angle = M_PI*static_cast<double>(i) / static_cast<double>(boxes);
      opt.modelPose.Pos().Set(radius*cos(angle), radius*sin(angle), dz/2);

      if (ring == 0)
        opt.direction1 = ignition::math::Vector3d(-sin(angle), cos(angle), 0);
      else
        opt.direction1 = ignition::math::Vector3d(0.0, 1.0, 0.0);

      if (ring == 1)
        opt.collisionPose.Rot().Euler(0.0, 0.0, angle);

      if (ring == 2)
        opt.linkPose.Rot().Euler(0.0, 0.0, angle);

      if (ring == 3)
        opt.modelPose.Rot().Euler(0.0, 0.0, angle);

      physics::ModelPtr model = SpawnBox(opt);
      ASSERT_TRUE(model != NULL);
      modelAngles[model] = angle;
    }
  }

  // Step forward
  world->Step(1500);
  double t = world->GetSimTime().Double();

  gzdbg << "Checking velocity after " << t << " seconds" << std::endl;
  std::map<physics::ModelPtr, double>::iterator iter;
  for (iter = modelAngles.begin(); iter != modelAngles.end(); ++iter)
  {
    double cosAngle = cos(iter->second);
    double sinAngle = sin(iter->second);
    double velMag = g.y * sinAngle * t;
    math::Vector3 vel = iter->first->GetWorldLinearVel();
    EXPECT_NEAR(velMag*cosAngle, vel.x, 5*g_friction_tolerance);
    EXPECT_NEAR(velMag*sinAngle, vel.y, 5*g_friction_tolerance);
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
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // set the gravity vector
  // small positive y component
  math::Vector3 g(0.0, 1.5, -1.0);
  physics->SetGravity(g);

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
  double t = world->GetSimTime().Double();

  gzdbg << "Checking velocity after " << t << " seconds" << std::endl;
  double velMag = (g.y+g.z) * t;
  math::Vector3 vel = model->GetWorldLinearVel();
  EXPECT_NEAR(0.0, vel.x, g_friction_tolerance);
  EXPECT_NEAR(velMag, vel.y, g_friction_tolerance);
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
