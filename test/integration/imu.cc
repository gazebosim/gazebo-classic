/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <ignition/math/Rand.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/test/helper_physics_generator.hh"

// How tightly to compare for deterministic values
#define IMU_TOL 1e-5

using namespace gazebo;
class ImuTest : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  /// \brief start imu_sensor_test.world, which contains a pendulum,
  /// a sphere with frictional contact, a sphere with frictionless
  /// contact and a ramp.  Each model has an IMU attached.
  /// This test check results to make sure the readings adhere to
  /// each simple model under gravity.
  public: void ImuSensorTestWorld(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs.
  public: void Stationary_EmptyWorld(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs with noise enabled.
  public: void Stationary_EmptyWorld_Noise(const std::string &_physicsEngine);

  /// \brief Spawn a static model with an ImuSensor attached
  /// in the empty world.  Test basic IMU outputs with bias enabled.
  public: void Stationary_EmptyWorld_Bias(const std::string &_physicsEngine);

  /// \brief Return gravity rotated by some orientation
  /// \param[in] _rot User specified rotation
  /// \param[out] _g gravity in user specified orientation
  private: void GetGravity(const ignition::math::Quaterniond &_rot,
                                 ignition::math::Vector3d &_g);

  /// \brief Collect a number of samples and return the average
  /// rate and accel values
  /// \param[in] _imu Pointer to sensor
  /// \param[in] _cnt number of samples to tak
  /// \param[out] _rateMean average angular rates over samples
  /// \param[out] _accelMean average accelerations over samples
  /// \param[out] _orientation orientation of the imu at the end of sample
  /// period
  private: void GetImuData(sensors::ImuSensorPtr _imu, unsigned int _cnt,
               ignition::math::Vector3d &_rateMean,
               ignition::math::Vector3d &_accelMean,
               ignition::math::Quaterniond &_orientation);
};

void ImuTest::GetGravity(const ignition::math::Quaterniond &_rot,
                               ignition::math::Vector3d &_g)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  // Rotate into IMU's frame
  _g = _rot.Inverse().RotateVector(world->Gravity());
}

void ImuTest::GetImuData(sensors::ImuSensorPtr _imu,
                         unsigned int _cnt,
                         ignition::math::Vector3d &_rateMean,
                         ignition::math::Vector3d &_accelMean,
                         ignition::math::Quaterniond &_orientation)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  // Collect a number of samples and return the average rate and accel values
  ignition::math::Vector3d rateSum, accelSum;
  for (unsigned int i = 0; i < _cnt; ++i)
  {
    world->Step(1);

    int j = 0;
    while (_imu->LastMeasurementTime() == gazebo::common::Time::Zero &&
        j < 100)
    {
      _imu->Update(true);
      gazebo::common::Time::MSleep(100);
      ++j;
    }

    EXPECT_LT(j, 100);

    rateSum += _imu->AngularVelocity();
    accelSum += _imu->LinearAcceleration();
  }
  _rateMean = rateSum / _cnt;
  _accelMean = accelSum / _cnt;
  _orientation = _imu->Orientation();
}

void ImuTest::ImuSensorTestWorld(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "not working yet for anything other than ode. see issue #893.\n";
    return;
  }

  Load("worlds/imu_sensor_test.world", true, _physicsEngine);

  // get world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // get pendulum
  std::string pendulumName = "model_pendulum";
  physics::ModelPtr pendulumModel = world->GetModel(pendulumName);
  ASSERT_TRUE(pendulumModel != NULL);

  std::string pendulumSensorName = "pendulum_imu_sensor";
  sensors::ImuSensorPtr pendulumImu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(pendulumSensorName));
  ASSERT_TRUE(pendulumImu != NULL);
  pendulumImu->Init();

  // get friction ball
  std::string ballFrictionName = "model_ball";
  physics::ModelPtr ballFrictionModel = world->GetModel(ballFrictionName);
  ASSERT_TRUE(ballFrictionModel != NULL);

  std::string ballFrictionSensorName = "ball_imu_sensor";
  sensors::ImuSensorPtr ballFrictionImu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFrictionSensorName));
  ASSERT_TRUE(ballFrictionImu != NULL);
  ballFrictionImu->Init();

  // get frictionless ball
  std::string ballNoFrictionName = "model_ball_no_friction";
  physics::ModelPtr ballNoFrictionModel = world->GetModel(ballNoFrictionName);
  ASSERT_TRUE(ballNoFrictionModel != NULL);

  std::string ballNoFrictionSensorName = "ball_no_friction_imu_sensor";
  sensors::ImuSensorPtr ballNoFrictionImu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballNoFrictionSensorName));
  ASSERT_TRUE(ballNoFrictionImu != NULL);
  ballNoFrictionImu->Init();

  // get floating ball
  std::string ballFloatingName = "model_floating_imu";
  physics::ModelPtr ballFloatingModel = world->GetModel(ballFloatingName);
  ASSERT_TRUE(ballFloatingModel != NULL);

  std::string ballFloatingSensorName = "ball_floating_imu_sensor";
  sensors::ImuSensorPtr ballFloatingImu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFloatingSensorName));
  ASSERT_TRUE(ballFloatingImu != NULL);
  ballFloatingImu->Init();

  // get floating ball 2
  std::string ballFloatingName2 = "link_floating_imu_2";
  physics::LinkPtr ballFloatingLink2 =
    ballFloatingModel->GetLink(ballFloatingName2);
  ASSERT_TRUE(ballFloatingLink2 != NULL);

  std::string ballFloatingSensorName2 = "ball_floating_imu_sensor_2";
  sensors::ImuSensorPtr ballFloatingImu2 =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFloatingSensorName2));
  ASSERT_TRUE(ballFloatingImu2 != NULL);
  ballFloatingImu2->Init();

  // get gravity
  auto g = world->Gravity();

  // run for 1900 steps (Step(1) each time), or 1.9 seconds, enough
  // to capture what we need from this experiment.
  for (unsigned n = 0; n < 1900; ++n)
  {
    world->Step(1);
    // gzdbg << "time: " << world->GetSimTime().Double() << "\n";

    // pendulum
    // on startup
    //   sensor linear accel [0 0 0]
    //   Link::GetRelativeLinearAccel() [0 0 -9.81]
    //   Link::GetWorldLinearAccel() [0 0 -9.81]
    // @T=1.872 sec, at max lowest position
    //   sensor linear accel [-0 -0.041216 29.4258]
    //   Link::GetRelativeLinearAccel() [-0 -0.008923 19.6159]
    //   Link::GetWorldLinearAccel() [-0 0.055649 19.6158]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        pendulumImu->LinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        pendulumModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        pendulumModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() == 1.872)
      {
        // initial values
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), -0.041216, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 29.42581726, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, -0.036397, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, 19.6158848, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, -0.0267709, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, 19.6159003, IMU_TOL);
      }
      else
      {
        // initial values
        EXPECT_LE(imuLinearAccel.Z(), 29.4259);
        EXPECT_LE(relativeLinearAccel.z, 19.616);
        EXPECT_LE(worldLinearAccel.z, 19.616);
      }
    }

    // friction ball
    // before contact
    //   sensor linear accel [0 0 0]
    //   Link::GetRelativeLinearAccel() [0 0 -9.81]
    //   Link::GetWorldLinearAccel() [0 0 -9.81]
    //
    // @T=1.2 sec, on ramp - varies, e.g.
    //   sensor linear accel [-7.81558 0 3.71003]
    //   Link::GetRelativeLinearAccel() [1.98569 0 3.29613]
    //   Link::GetWorldLinearAccel() [3.37698 0 -1.84485]
    //
    // @T=1.849 sec, on ground - sensor vector rotates
    //   sensor linear accel [-2.93844 0 9.35958]
    //   Link::GetRelativeLinearAccel() [0 0 0]
    //   Link::GetWorldLinearAccel() [0 0 0]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        ballFrictionImu->LinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        ballFrictionModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        ballFrictionModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, g.X(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, g.Y(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, g.Z(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, g.X(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, g.Y(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, g.Z(), IMU_TOL);
      }
      // should use contact detector for these timing stuff
      else if (world->GetSimTime().Double() >= 1.2 &&
               world->GetSimTime().Double() <= 1.84)
      {
        // on ramp
        // ...hm, not much can be said in simple terms, leave out for now.
      }
      else if (world->GetSimTime().Double() >= 1.85)
      {
        // on the ground
        double imuMag = imuLinearAccel.Length();
        double gMag = g.Length();
        EXPECT_NEAR(imuMag, gMag, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, 0, IMU_TOL);
      }
    }

    // frictionless ball
    // before contact
    //   sensor linear accel [0 0 0]
    //   Link::GetRelativeLinearAccel() [0 0 -9.81]
    //   Link::GetWorldLinearAccel() [0 0 -9.81]
    // @T=1.2 sec, on ramp - constant
    //   sensor linear accel [4.12742 0 7.55518]
    //   Link::GetRelativeLinearAccel() [4.12742 0 -2.25482]
    //   Link::GetWorldLinearAccel() [4.12742 0 -2.25482]
    // @T=1.8 sec, on ground - constant
    //   sensor linear accel [0 0 9.81]
    //   Link::GetRelativeLinearAccel() [0 0 0]
    //   Link::GetWorldLinearAccel() [0 0 0]
    {
      // get states from imu sensor
      ignition::math::Vector3d imuLinearAccel =
        ballNoFrictionImu->LinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        ballNoFrictionModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        ballNoFrictionModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.X(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Y(), 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.Z(), 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, g.X(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, g.Y(), IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, g.Z(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, g.X(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, g.Y(), IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, g.Z(), IMU_TOL);
      }
      else if (world->GetSimTime().Double() >= 1.3 &&
               world->GetSimTime().Double() <= 1.751)
      {
        // on the ramp
        const double rampAngle = 0.5;
        double gMag = g.Length();
        double imuMag = imuLinearAccel.Length();
        EXPECT_NEAR(imuMag, gMag*cos(rampAngle), IMU_TOL);

        double relMag = relativeLinearAccel.GetLength();
        EXPECT_NEAR(relMag, gMag*sin(rampAngle), IMU_TOL);
        double worMag = worldLinearAccel.GetLength();
        EXPECT_NEAR(worMag, gMag*sin(rampAngle), IMU_TOL);
      }
      else if (world->GetSimTime().Double() >= 1.8)
      {
        // on the ground
        double imuMag = imuLinearAccel.Length();
        double gMag = g.Length();
        EXPECT_NEAR(imuMag, gMag, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, 0, IMU_TOL);
      }
    }
  }

  // floating ball
  // This "robot" starts up aligned with world axis.
  // test that SetReferencePose resets orientation to identity
  ballFloatingImu->SetReferencePose();
  ignition::math::Quaterniond imuOrientation = ballFloatingImu->Orientation();
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.W(),
      imuOrientation.W(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.X(),
      imuOrientation.X(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.Y(),
      imuOrientation.Y(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.Z(),
      imuOrientation.Z(), IMU_TOL);

  // test that SetReferenceOrientation sets orientation to argument
  // in this test case, assume world is NWU (X-North, Y-West, Z-Up),
  // then transform from NWU to NED is below:
  ignition::math::Quaterniond nwuToNEDReference =
    ignition::math::Quaterniond(M_PI, 0, 0);
  // declare NED frame the reference frame for this IMU
  ballFloatingImu->SetWorldToReferenceOrientation(nwuToNEDReference);

  // let messages propagate asynchronously
  world->Step(1000);

  /****************************************************************************/
  /*                                                                          */
  /*                     Static Pose Initialization Tests                     */
  /*                                                                          */
  /****************************************************************************/
  /* orientation of the imu in NED frame                                      */
  /****************************************************************************/
  imuOrientation = ballFloatingImu->Orientation();

  EXPECT_NEAR(imuOrientation.W(), 0, IMU_TOL);
  EXPECT_NEAR(imuOrientation.X(), -1, IMU_TOL);
  EXPECT_NEAR(imuOrientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(imuOrientation.Z(), 0, IMU_TOL);

  // imu orientation in world frame
  ignition::math::Quaterniond imuWorldOrientation =
    imuOrientation * nwuToNEDReference;

  EXPECT_NEAR(imuWorldOrientation.W(), 1, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.Z(), 0, IMU_TOL);

  /****************************************************************************/
  /* floating ball 2                                                          */
  /* This "robot" starts with a yaw of 1.8 rad from world frame.              */
  /* test that SetReferencePose resets orientation to identity                */
  /****************************************************************************/
  ballFloatingImu2->SetReferencePose();
  ignition::math::Quaterniond imuOrientation2 = ballFloatingImu2->Orientation();
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.W(),
      imuOrientation2.W(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.X(),
      imuOrientation2.X(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.Y(),
      imuOrientation2.Y(), IMU_TOL);
  EXPECT_NEAR(ignition::math::Quaterniond::Identity.Z(),
      imuOrientation2.Z(), IMU_TOL);

  /****************************************************************************/
  /*                                                                          */
  /*                     Static Pose Manipulation Tests                       */
  /*                                                                          */
  /****************************************************************************/
  /* test that SetReferenceOrientation sets orientation to argument           */
  /* in this test case, assume world is NWU (X-North, Y-West, Z-Up),          */
  /* then transform from NWU to NED is below:                                 */
  /****************************************************************************/
  // declare NED frame the reference frame for this IMU
  ballFloatingImu2->SetWorldToReferenceOrientation(nwuToNEDReference);

  // let messages propagate asynchronously
  world->Step(1000);

  // orientation of the imu in NED frame
  const double imu2Angle = 1.8;
  imuOrientation2 = ballFloatingImu2->Orientation();
  ignition::math::Vector3d rpy2 = imuOrientation2.Euler();
  EXPECT_NEAR(fabs(rpy2.X()), M_PI, IMU_TOL);
  EXPECT_NEAR(rpy2.Y(), 0, IMU_TOL);
  EXPECT_NEAR(rpy2.Z(), -imu2Angle, IMU_TOL);

  // imu orientation in world frame
  ignition::math::Quaterniond imuWorldOrientation2 =
    imuOrientation2 * nwuToNEDReference;
  ignition::math::Vector3d rpyWorld2 = imuWorldOrientation2.Euler();
  EXPECT_NEAR(rpyWorld2.X(), 0, IMU_TOL);
  EXPECT_NEAR(rpyWorld2.Y(), 0, IMU_TOL);
  EXPECT_NEAR(rpyWorld2.Z(), -imu2Angle, IMU_TOL);

  /****************************************************************************/
  /*                                                                          */
  /*                     Kinematic Pose Manipulation Test                     */
  /*                                                                          */
  /****************************************************************************/
  /* turn floating ball 2 by -1.8 rad yaw, and see if two floating            */
  /* balls orientation match                                                  */
  /****************************************************************************/
  ballFloatingLink2->SetWorldPose(
      ignition::math::Pose3d(3.0, -3.40, 0.95, 0.0, 0.0, 0.0));

  // let messages propagate asynchronously
  world->Step(1000);

  // get orientation of two floating balls and compare them
  imuOrientation = ballFloatingImu->Orientation();
  ignition::math::Vector3d rpy = imuOrientation.Euler();

  imuOrientation2 = ballFloatingImu2->Orientation();
  rpy2 = imuOrientation2.Euler();

  EXPECT_NEAR(fabs(rpy.X()), fabs(rpy2.X()), IMU_TOL);
  EXPECT_NEAR(rpy.Y(), rpy2.Y(), IMU_TOL);
  EXPECT_NEAR(rpy.Z(), rpy2.Z(), IMU_TOL);

  /****************************************************************************/
  /*                                                                          */
  /*                     Dynamic Torque Test                                  */
  /*                                                                          */
  /****************************************************************************/
  /* turn floating ball 2 by 0.6 rad yaw, apply torque about imu local Y-axis */
  /* and test AngularVelocity is expressed in local frame.                    */
  /****************************************************************************/
  const double yaw = 0.6;
  ballFloatingLink2->SetWorldPose(
      ignition::math::Pose3d(3.0, -3.40, 0.95, 0.0, 0.0, yaw));

  // HACK: take 100 steps, due to message passing synchronization delays,
  // we should fix this by haing an equivalent blocking "service call" to
  // SetWorldPose.
  world->Step(100);

  // Nm
  const double tau = 150.0;

  ballFloatingLink2->AddRelativeTorque(
      ignition::math::Vector3d(0, tau, 0));

  // expected velocity calculation
  // kg*m^2
  const double iyy = ballFloatingLink2->GetInertial()->GetPrincipalMoments().y;
  EXPECT_NEAR(iyy, 0.1, 1e-6);

  // sec
  const double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_NEAR(dt, 0.001, 1e-6);

  // 1.5 m/s , pitch rate
  const double pDot = tau / iyy * dt;
  const int nsteps = 1000;
  const double p = pDot * (nsteps-1) * dt;

  // let messages propagate asynchronously
  world->Step(nsteps);

  // get orientation of two floating balls and compare them
  imuOrientation2 = ballFloatingImu2->Orientation();
  rpy2 = imuOrientation2.Euler();
  ignition::math::Vector3d rpyDot2 = ballFloatingImu2->AngularVelocity();
  ignition::math::Vector3d linAcc2 = ballFloatingImu2->LinearAcceleration();

  // compare against analytical results
  // because NED
  EXPECT_NEAR(fabs(rpy2.X()), M_PI, IMU_TOL);
  EXPECT_NEAR(rpy2.Y(), -p, IMU_TOL);
  EXPECT_NEAR(rpy2.Z(), -yaw, IMU_TOL);

  EXPECT_NEAR(rpyDot2.X(), 0, IMU_TOL);
  EXPECT_NEAR(rpyDot2.Y(), pDot, IMU_TOL);
  EXPECT_NEAR(rpyDot2.Z(), 0, IMU_TOL);

  // centripetal acceleration along radial (x-axis) direction.
  // then add gravity acceleration to x and z based on orientation of imu.
  this->GetGravity(ballFloatingImu2->Orientation(), g);

  const double radius = boost::static_pointer_cast<physics::SphereShape>(
      ballFloatingLink2 ->GetCollision(
        "collision_sphere")->GetShape())->GetRadius();

  // const double acc = -pDot*pDot*r;
  const double acc = -rpyDot2.Y()*rpyDot2.Y()*radius;
  const double accX = acc + g.X();
  const double accZ = g.Z();
  EXPECT_NEAR(linAcc2.X(), accX, IMU_TOL);
  EXPECT_NEAR(linAcc2.Y(), 0, IMU_TOL);

  // FIXME: why is this error larger than default tol 1e-5?
  // See ign-math issue #47.
  // https://bitbucket.org/ignitionrobotics/ign-math/issues/47
  const double special_IMU_TOL = 0.00016874990503534804;
  EXPECT_NEAR(linAcc2.Z(), accZ, special_IMU_TOL);

  /****************************************************************************/
  /*                                                                          */
  /*                     Dynamic Linear Force Test                            */
  /*                                                                          */
  /****************************************************************************/
  /* turn floating ball 2 by 0.5 rad pitch, apply force about                 */
  /* positive world z-axis                                                    */
  /* and test if LinearAcceleration is expressed in local frame.              */
  /****************************************************************************/
  const double pitch = 0.5;
  ballFloatingImu2->SetWorldToReferenceOrientation(
    ignition::math::Quaterniond());
  ballFloatingLink2->Reset();
  ballFloatingLink2->SetWorldPose(
      ignition::math::Pose3d(3.0, -3.40, 0.95, 0.0, pitch, 0.0));

  world->Step(100);
  for (int i = 0; i < 1000; ++i)
  {
    const double f = 13.8;
    ballFloatingLink2->AddForce(ignition::math::Vector3d(0, 0, f));
    world->Step(1);

    const double m = 5.0;
    const double a = f / m;

    ignition::math::Vector3d linAcc2 = ballFloatingImu2->LinearAcceleration();
    this->GetGravity(ballFloatingImu2->Orientation(), g);

    if (i > 100)
    {
      // THERE MUST BE A BETTER WAY TO DO THIS...
      // need to take 100 stesps to ensure that
      // imu readings are passed through from asynchronous transport
      EXPECT_NEAR(linAcc2.X(), -a*sin(pitch) - g.X(), IMU_TOL);
      EXPECT_NEAR(linAcc2.Y(), 0 - g.Y(), IMU_TOL);
      EXPECT_NEAR(linAcc2.Z(), a*cos(pitch) - g.Z(), IMU_TOL);
    }
  }
}

TEST_P(ImuTest, ImuSensorTestWorld)
{
  ImuSensorTestWorld(GetParam());
}

void ImuTest::Stationary_EmptyWorld(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(0.5, -1.0, 0.2));

  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler());

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->GetImuData(imu, 1, rateMean, accelMean, orientation);

  EXPECT_NEAR(rateMean.X(), 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.Y(), 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.Z(), 0.0, IMU_TOL);

  ignition::math::Vector3d g;
  this->GetGravity(testPose.Rot(), g);
  EXPECT_NEAR(accelMean.X(), -g.X(), IMU_TOL);
  EXPECT_NEAR(accelMean.Y(), -g.Y(), IMU_TOL);
  EXPECT_NEAR(accelMean.Z(), -g.Z(), IMU_TOL);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorld)
{
  Stationary_EmptyWorld(GetParam());
}

void ImuTest::Stationary_EmptyWorld_Noise(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(0.3, -1.4, 2.0));

  double rateNoiseMean = 1.0;
  double rateNoiseStddev = 0.1;
  double rateBiasMean = 0.0;
  double rateBiasStddev = 0.0;
  double accelNoiseMean = -10.0;
  double accelNoiseStddev = 0.1;
  double accelBiasMean = 0.0;
  double accelBiasStddev = 0.0;
  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.X() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.X() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Y() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Y() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Z() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Z() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  ignition::math::Vector3d g;
  this->GetGravity(testPose.Rot(), g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.X() - (accelNoiseMean + accelBiasMean) + g.X());
  d2 = fabs(accelMean.X() - (accelNoiseMean - accelBiasMean) + g.X());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Y() - (accelNoiseMean + accelBiasMean) + g.Y());
  d2 = fabs(accelMean.Y() - (accelNoiseMean - accelBiasMean) + g.Y());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Z() - (accelNoiseMean + accelBiasMean) + g.Z());
  d2 = fabs(accelMean.Z() - (accelNoiseMean - accelBiasMean) + g.Z());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldNoise)
{
  Stationary_EmptyWorld_Noise(GetParam());
}

void ImuTest::Stationary_EmptyWorld_Bias(const std::string &_physicsEngine)
{
  // static models not fully working in simbody yet
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test for Simbody, see issue #860.\n";
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "imu_model";
  std::string imuSensorName = "imu_sensor";
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 0.05),
      ignition::math::Quaterniond(-0.3, 0.5, 1.0));

  double rateNoiseMean = 0.0;
  double rateNoiseStddev = 0.0;
  double rateBiasMean = 1.0;
  double rateBiasStddev = 0.1;
  double accelNoiseMean = 0.0;
  double accelNoiseStddev = 0.0;
  double accelBiasMean = 5.0;
  double accelBiasStddev = 0.1;
  SpawnImuSensor(modelName, imuSensorName, testPose.Pos(),
      testPose.Rot().Euler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu != NULL);
  imu->Init();
  ignition::math::Vector3d rateMean, accelMean;
  ignition::math::Quaterniond orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.X() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.X() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Y() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Y() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.Z() - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.Z() - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  ignition::math::Vector3d g;
  this->GetGravity(testPose.Rot(), g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.X() - (accelNoiseMean + accelBiasMean) + g.X());
  d2 = fabs(accelMean.X() - (accelNoiseMean - accelBiasMean) + g.X());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Y() - (accelNoiseMean + accelBiasMean) + g.Y());
  d2 = fabs(accelMean.Y() - (accelNoiseMean - accelBiasMean) + g.Y());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.Z() - (accelNoiseMean + accelBiasMean) + g.Z());
  d2 = fabs(accelMean.Z() - (accelNoiseMean - accelBiasMean) + g.Z());
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.X(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Y(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.Z(), 0, IMU_TOL);
  EXPECT_NEAR(orientation.W(), 1, IMU_TOL);
}

TEST_P(ImuTest, EmptyWorldBias)
{
  Stationary_EmptyWorld_Bias(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
