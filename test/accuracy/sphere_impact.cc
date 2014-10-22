/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

using namespace gazebo;

// physics engine
// dt
// number of iterations
// inertia of sphere
// size of sphere
// gravity applied
// velocity of sphere
// lcp tolerance
// cg z offset
typedef std::tr1::tuple<const char * /* physics engine */
                      , int          /* number of iterations */
                      , double       /* dt */
                      , double       /* mass of sphere */
                      , double       /* radius of sphere */
                      , double       /* gravity */
                      , double       /* velocity on sphere */
                      , double       /* tolerance */
                      , double       /* cg z offset */
                      > char1int1double4;
class RigidBodyTest : public ServerFixture,
                      public testing::WithParamInterface<char1int1double4>
{
  /// \brief Test accuracy of unconstrained rigid body motion.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _iterations Number of iterations.
  /// \param[in] _dt Max time step size.
  /// \param[in] _mass Mass sphere
  /// \param[in] _radius Size of sphere
  /// \param[in] _gravity gravity applied
  /// \param[in] _velocity velocity of sphere
  /// \param[in] _cgz cg offset
  public: void SphereImpact(const std::string &_physicsEngine
                          , int _iterations
                          , double _dt
                          , double _mass
                          , double _radius
                          , double _gravity
                          , double _velocity
                          , double _tolerance
                          , double _cgz
                          );
};

/////////////////////////////////////////////////
// SphereImpact:
// impact sphere and record accuracy for momentum and enery
// conservation
void RigidBodyTest::SphereImpact(const std::string &_physicsEngine
                               , int _iterations
                               , double _dt
                               , double _mass
                               , double _radius
                               , double _gravity
                               , double _velocity
                               , double _tolerance
                               , double _cgz
                               )
{
  // Load a blank world (no ground plane)
  Load("worlds/empty_physics.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // get model and link
  physics::Model_V models;

  // get gravity value
  physics->SetGravity(math::Vector3(0, 0, _gravity));

  math::Vector3 g = physics->GetGravity();

  // set simulation time step size
  //   change step size after setting initial conditions
  //   since simbody requires a time step
  physics->SetMaxStepSize(_dt);
  if (_physicsEngine == "ode" || _physicsEngine == "bullet")
  {
    gzdbg << "iters: "
          << boost::any_cast<int>(physics->GetParam("iters"))
          << std::endl;
    physics->SetParam("iters", _iterations);
    physics->SetParam("sor_lcp_tolerance", _tolerance);
  }

  // unthrottle update rate
  physics->SetRealTimeUpdateRate(0.0);

  // Create 2 spheres with inertia based on sphere of uniform density
  {
    // first sphere at (-5, 0, 1)m, with mass and radius specified
    // by test parameters.
    msgs::Model msgModel;
    msgModel.set_name("sphere_1");

    msgModel.add_link();
    msgs::Link *msgLink = msgModel.mutable_link(0);
    msgLink->set_name("link");
    msgs::Inertial *msgInertial = msgLink->mutable_inertial();
    msgInertial->set_mass(_mass);
    double ixx = 2.0 * _mass * _radius * _radius / 5.0;
    msgInertial->set_ixx(ixx);
    msgInertial->set_ixy(0.0);
    msgInertial->set_ixz(0.0);
    msgInertial->set_iyy(ixx);
    msgInertial->set_iyz(0.0);
    msgInertial->set_izz(ixx);
    msgInertial->mutable_pose()->mutable_position()->set_x(0.0);
    msgInertial->mutable_pose()->mutable_position()->set_y(0.0);
    msgInertial->mutable_pose()->mutable_position()->set_z(_cgz);

    msgLink->add_collision();
    msgs::Collision *msgCollision = msgLink->mutable_collision(0);
    msgCollision->set_name("collision");
    msgs::Geometry *msgGeometry = msgCollision->mutable_geometry();
    msgGeometry->set_type(msgs::Geometry_Type_SPHERE);
    msgGeometry->mutable_sphere()->set_radius(_radius);

    // add visual doesn't work
    // msgLink->add_visual();
    // msgs::Visual *msgVisual = msgLink->mutable_visual(0);
    // msgVisual->set_name("visual");
    // msgGeometry = msgVisual->mutable_geometry();
    // msgGeometry->set_type(msgs::Geometry_Type_SPHERE);
    // msgGeometry->mutable_sphere()->set_radius(_radius);

    // 1m above ground, -5 meters away from origin
    math::Vector3 pos(-5.0 - _radius, 0.0, 1.0 + _radius);
    msgs::Set(msgModel.mutable_pose()->mutable_position(), pos);

    models.push_back(this->SpawnModel(msgModel));
  }
  {
    msgs::Model msgModel;
    msgModel.set_name("sphere_2");

    msgModel.add_link();
    msgs::Link *msgLink = msgModel.mutable_link(0);
    msgLink->set_name("link");
    msgs::Inertial *msgInertial = msgLink->mutable_inertial();
    const double mass2 = 1.0;
    const double radius2 = 0.5;
    msgInertial->set_mass(mass2);
    double ixx = 2.0 * mass2 * radius2 * radius2 / 5.0;
    msgInertial->set_ixx(ixx);
    msgInertial->set_ixy(0.0);
    msgInertial->set_ixz(0.0);
    msgInertial->set_iyy(ixx);
    msgInertial->set_iyz(0.0);
    msgInertial->set_izz(ixx);
    msgInertial->mutable_pose()->mutable_position()->set_x(0.0);
    msgInertial->mutable_pose()->mutable_position()->set_y(0.0);
    msgInertial->mutable_pose()->mutable_position()->set_z(_cgz);

    msgLink->add_collision();
    msgs::Collision *msgCollision = msgLink->mutable_collision(0);
    msgCollision->set_name("collision");
    msgs::Geometry *msgGeometry = msgCollision->mutable_geometry();
    msgGeometry->set_type(msgs::Geometry_Type_SPHERE);
    msgGeometry->mutable_sphere()->set_radius(radius2);

    // add visual doesn't work
    // msgLink->add_visual();
    // msgs::Visual *msgVisual = msgLink->mutable_visual(0);
    // msgVisual->set_name("visual");
    // msgGeometry = msgVisual->mutable_geometry();
    // msgGeometry->set_type(msgs::Geometry_Type_SPHERE);
    // msgGeometry->mutable_sphere()->set_radius(radius2);

    // 1m above ground, 0m way from origin
    math::Vector3 pos(0.0 + radius2, 0.0, 1.0 + _radius);
    msgs::Set(msgModel.mutable_pose()->mutable_position(), pos);

    models.push_back(this->SpawnModel(msgModel));
  }

  // get top model and link
  physics::ModelPtr model_1 = world->GetModel("sphere_1");
  physics::LinkPtr link_1 = model_1->GetLink("link");
  physics::ModelPtr model_2 = world->GetModel("sphere_2");
  physics::LinkPtr link_2 = model_2->GetLink("link");


  // set things in motion for a few time steps here
  // e.g. start spheres moving
  // be sure to stop before collision happens
  // setup simulation duration
  const double startupDuration = 10.0;
  int startupSteps = ceil(startupDuration / _dt);

  for (int i = 0; i < startupSteps; ++i)
  {
    // apply velocity to top link
    link_1->SetLinearVel(math::Vector3(_velocity, 0.0, 0));
    // link_2->SetLinearVel(math::Vector3(_velocity, 0.0, 0));

    // step world once
    world->Step(1);
    double dist =
      (link_1->GetWorldPose() - link_2->GetWorldPose()).pos.GetLength();

    // if spheres are about to collide
    if (dist < 0.1) // _velocity * _dt)
      break;
  }



  // initial time
  common::Time t0 = world->GetSimTime();

  // initial linear velocity in global frame
  const math::Vector3 v0_1 = link_1->GetWorldLinearVel();
  const math::Vector3 v0_2 = link_2->GetWorldLinearVel();

  // initial linear momentum in global frame
  math::Vector3 P0_1 = link_1->GetInertial()->GetMass() * v0_1;
  double P0mag_1 = P0_1.GetLength();
  math::Vector3 P0_2 = link_2->GetInertial()->GetMass() * v0_2;
  double P0mag_2 = P0_2.GetLength();

  // initial angular velocity in global frame
  math::Vector3 w0_1 = link_1->GetWorldAngularVel();
  math::Vector3 w0_2 = link_2->GetWorldAngularVel();

  // initial angular momentum in global frame
  math::Vector3 H0_1 = link_1->GetWorldInertiaMatrix() * w0_1;
  double H0mag_1 = H0_1.GetLength();
  math::Vector3 H0_2 = link_2->GetWorldInertiaMatrix() * w0_2;
  double H0mag_2 = H0_2.GetLength();

  // initial energy
  double E0_1 = link_1->GetWorldEnergy();
  double E0_2 = link_2->GetWorldEnergy();

  // variables to compute statistics on
  math::Vector3Stats linearMomentumError;
  math::Vector3Stats angularMomentumError;
  math::SignalStats energyError;
  math::SignalStats constraintErrorTotal;
  math::SignalStats constraintResidualTotal;
  {
    const std::string statNames = "MaxAbs,Variance,Mean";
    EXPECT_TRUE(linearMomentumError.InsertStatistics(statNames));
    EXPECT_TRUE(angularMomentumError.InsertStatistics(statNames));
    EXPECT_TRUE(energyError.InsertStatistics(statNames));
    EXPECT_TRUE(constraintErrorTotal.InsertStatistics(statNames));
    EXPECT_TRUE(constraintResidualTotal.InsertStatistics(statNames));
  }

  // setup simulation duration
  const double simDuration = 10.0;
  int simSteps = ceil(simDuration / _dt);

  // record start wall time
  common::Time startWallTime = common::Time::GetWallTime();
  for (int i = 0; i < simSteps; ++i)
  {
    // step world once
    world->Step(1);

    // current time
    // double t = (world->GetSimTime() - t0).Double();

    // linear velocity error
    math::Vector3 v_1 = link_1->GetWorldCoGLinearVel();
    math::Vector3 v_2 = link_2->GetWorldCoGLinearVel();

    // linear momentum error
    math::Vector3 P_1 = link_1->GetInertial()->GetMass()*v_1;
    math::Vector3 P_2 = link_2->GetInertial()->GetMass()*v_2;
    linearMomentumError.InsertData((P_1 + P_2 - P0_1 - P0_2)
      / (P0mag_1 + P0mag_2));

    // angular momentum error
    math::Vector3 H_1 =
      link_1->GetWorldInertiaMatrix()*link_1->GetWorldAngularVel();
    math::Vector3 H_2 =
      link_2->GetWorldInertiaMatrix()*link_2->GetWorldAngularVel();
    angularMomentumError.InsertData((H_1 + H_2 - H0_1 - H0_2)
      / (H0mag_1 + H0mag_2));

    // energy error
    energyError.InsertData(
      (link_1->GetWorldEnergy() + link_2->GetWorldEnergy() - E0_1 - E0_2)
      / (E0_1 + E0_2));

    // extended test for ode
    if (_physicsEngine == "ode")
    {
      double * rmsError =
        boost::any_cast<double*>(physics->GetParam("rms_error"));
      double * residual =
        boost::any_cast<double*>(physics->GetParam("constraint_residual"));

      constraintErrorTotal.InsertData(rmsError[3]);
      constraintResidualTotal.InsertData(residual[3]);
      // gzerr << "residual "
      //       << "[" <<  residual[0]
      //       << ", " <<  residual[1]
      //       << ", " <<  residual[2]
      //       << ", " <<  residual[3]
      //       << "]\n";
    }
  }
  common::Time elapsedTime = common::Time::GetWallTime() - startWallTime;
  this->Record("wallTime", elapsedTime.Double());
  common::Time simTime = (world->GetSimTime() - t0).Double();
  ASSERT_NEAR(simTime.Double(), simDuration, _dt*1.1);
  this->Record("simTime", simTime.Double());
  this->Record("timeRatio", elapsedTime.Double() / simTime.Double());

  // Record statistics on pitch and yaw angles
  this->Record("linMomentum0", (P0mag_1 + P0mag_2));
  this->Record("linMomentumError", linearMomentumError.Mag());
  this->Record("angMomentum0", (H0mag_1 + H0mag_2));
  this->Record("angMomentumError", angularMomentumError.Mag());
  this->Record("energy0", (E0_1 + E0_2));
  this->Record("energyError", energyError);
  this->Record("rmsErrorTotal", constraintErrorTotal);
  this->Record("constraintResidualTotal", constraintResidualTotal);
}

/////////////////////////////////////////////////
TEST_P(RigidBodyTest, SphereImpact)
{
  std::string physicsEngine = std::tr1::get<0>(GetParam());
  int    iterations         = std::tr1::get<1>(GetParam());
  double dt                 = std::tr1::get<2>(GetParam());
  double mass               = std::tr1::get<3>(GetParam());
  double radius             = std::tr1::get<4>(GetParam());
  double gravity            = std::tr1::get<5>(GetParam());
  double velocity           = std::tr1::get<6>(GetParam());
  double tolerance          = std::tr1::get<7>(GetParam());
  double cgz                = std::tr1::get<8>(GetParam());
  gzdbg << physicsEngine
        << ", dt: " << dt
        << ", iters: " << iterations
        << ", mass: " << mass
        << ", radius: " << radius
        << ", gravity: " << gravity
        << ", velocity: " << velocity
        << ", tolerance: " << tolerance
        << ", cgz: " << cgz
        << std::endl;
  RecordProperty("engine", physicsEngine);
  RecordProperty("iters", iterations);
  this->Record("dt", dt);
  this->Record("mass", mass);
  this->Record("radius", radius);
  this->Record("gravity", gravity);
  this->Record("velocity", velocity);  // negate, easier for plotting
  this->Record("tolerance", tolerance);
  this->Record("cgz", cgz);
  SphereImpact(physicsEngine
      , iterations
      , dt
      , mass
      , radius
      , gravity
      , velocity
      , tolerance
      , cgz
      );
}

// this test will vary mass of the sphere, and check for quality of solution
INSTANTIATE_TEST_CASE_P(SphereImpactMass, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(50)  // iterations
  , ::testing::Values(0.001) // step size
  // masses
  , ::testing::Values(1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0)
  , ::testing::Values(0.5) // radius
  , ::testing::Values(0.0) // gravity
  , ::testing::Values(1.0) // velocity
  , ::testing::Values(0.0) // tolerance
  , ::testing::Values(0.0) // cgz
  ));

// this test will vary size of the sphere, and check for quality of solution
INSTANTIATE_TEST_CASE_P(SphereImpactVelocity, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(50)  // iterations
  , ::testing::Values(0.001) // step size
  , ::testing::Values(1.0) // mass
  , ::testing::Values(0.5) // radius
  , ::testing::Values(0.0) // gravity
  , ::testing::Values(1.0, 10.0, 100.0, 1000.0) // velocity
  , ::testing::Values(0.0) // tolerance
  , ::testing::Values(0.0) // cgz
  ));

#define CGZ_MIN 0.0
#define CGZ_MAX 1.0
#define CGZ_STEP 0.05
// this test will vary cg of the sphere, and check for quality of solution
INSTANTIATE_TEST_CASE_P(SphereImpactCg, RigidBodyTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES
  , ::testing::Values(50)  // iterations
  , ::testing::Values(0.001) // step size
  , ::testing::Values(1.0) // mass
  , ::testing::Values(1.5) // radius
  , ::testing::Values(0.0) // gravity
  , ::testing::Values(1.0) // velocity
  , ::testing::Values(0.0) // tolerance
  , ::testing::Range(CGZ_MIN, CGZ_MAX, CGZ_STEP) // cgz
  ));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
