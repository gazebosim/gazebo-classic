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

#include "ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "helper_physics_generator.hh"

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

  /// \breif Return gravity rotated by some orientation
  /// \param[in] _rot User specified rotation
  /// \param[out] _g gravity in user specified orientation
  private: void GetGravity(const math::Quaternion& _rot, math::Vector3 &_g);

  /// \breif Collect a number of samples and return the average
  /// rate and accel values
  /// \param[in] _imu Pointer to sensor
  /// \param[in] _cnt number of samples to tak
  /// \param[out] _rateMean average angular rates over samples
  /// \param[out] _accelMean average accelerations over samples
  /// \param[out] _orientation orientation of the imu at the end of sample
  /// period
  private: void GetImuData(sensors::ImuSensorPtr _imu, unsigned int _cnt,
                           math::Vector3 &_rateMean,
                           math::Vector3 &_accelMean,
                           math::Quaternion &_orientation);
};

void ImuTest::GetGravity(const math::Quaternion &_rot, math::Vector3 &_g)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics);
  // Rotate into IMU's frame
  _g = _rot.GetInverse().RotateVector(physics->GetGravity());
}

void ImuTest::GetImuData(sensors::ImuSensorPtr _imu,
                         unsigned int _cnt,
                         math::Vector3 &_rateMean,
                         math::Vector3 &_accelMean,
                         math::Quaternion& _orientation)
{
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);
  // Collect a number of samples and return the average rate and accel values
  math::Vector3 rateSum, accelSum;
  for (unsigned int i = 0; i < _cnt; ++i)
  {
    world->Step(1);

    int j = 0;
    while (_imu->GetLastMeasurementTime() == gazebo::common::Time::Zero &&
        j < 100)
    {
      _imu->Update(true);
      gazebo::common::Time::MSleep(100);
      ++j;
    }

    EXPECT_LT(j, 100);

    rateSum += _imu->GetAngularVelocity();
    accelSum += _imu->GetLinearAcceleration();
  }
  _rateMean = rateSum / _cnt;
  _accelMean = accelSum / _cnt;
  _orientation = _imu->GetOrientation();
}

void ImuTest::ImuSensorTestWorld(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "not working yet for anything other than ode. see issue #9999.\n";
    return;
  }

  Load("worlds/imu_sensor_test.world", true, _physicsEngine);

  // get world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // get physics engine
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics);

  // get pendulum
  std::string pendulumName = "model_pendulum";
  physics::ModelPtr pendulumModel = world->GetModel(pendulumName);
  ASSERT_TRUE(pendulumModel);

  std::string pendulumSensorName = "pendulum_imu_sensor";
  sensors::ImuSensorPtr pendulumImu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(pendulumSensorName));
  ASSERT_TRUE(pendulumImu);
  pendulumImu->Init();

  // get friction ball
  std::string ballFrictionName = "model_ball";
  physics::ModelPtr ballFrictionModel = world->GetModel(ballFrictionName);
  ASSERT_TRUE(ballFrictionModel);

  std::string ballFrictionSensorName = "ball_imu_sensor";
  sensors::ImuSensorPtr ballFrictionImu =
    boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFrictionSensorName));
  ASSERT_TRUE(ballFrictionImu);
  ballFrictionImu->Init();

  // get frictionless ball
  std::string ballNoFrictionName = "model_ball_no_friction";
  physics::ModelPtr ballNoFrictionModel = world->GetModel(ballNoFrictionName);
  ASSERT_TRUE(ballNoFrictionModel);

  std::string ballNoFrictionSensorName = "ball_no_friction_imu_sensor";
  sensors::ImuSensorPtr ballNoFrictionImu =
    boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballNoFrictionSensorName));
  ASSERT_TRUE(ballNoFrictionImu);
  ballNoFrictionImu->Init();

  // get floating ball
  std::string ballFloatingName = "model_floating_imu";
  physics::ModelPtr ballFloatingModel = world->GetModel(ballFloatingName);
  ASSERT_TRUE(ballFloatingModel != NULL);

  std::string ballFloatingSensorName = "ball_floating_imu_sensor";
  sensors::ImuSensorPtr ballFloatingImu =
    boost::static_pointer_cast<sensors::ImuSensor>(
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
    boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(ballFloatingSensorName2));
  ASSERT_TRUE(ballFloatingImu2 != NULL);
  ballFloatingImu2->Init();

  // get gravity
  math::Vector3 g = physics->GetGravity();

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
      math::Vector3 imuLinearAccel = pendulumImu->GetLinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        pendulumModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        pendulumModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() == 1.872)
      {
        // initial values
        EXPECT_NEAR(imuLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.y, -0.041216, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.z, 29.42581726, IMU_TOL);
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
        EXPECT_LE(imuLinearAccel.z, 29.4259);
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
      math::Vector3 imuLinearAccel = ballFrictionImu->GetLinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        ballFrictionModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        ballFrictionModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.z, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, g.x, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, g.y, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, g.z, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, g.x, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, g.y, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, g.z, IMU_TOL);
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
        double imuMag = imuLinearAccel.GetLength();
        double gMag = g.GetLength();
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
      math::Vector3 imuLinearAccel =
        ballNoFrictionImu->GetLinearAcceleration();
      // get states from link
      math::Vector3 relativeLinearAccel =
        ballNoFrictionModel->GetRelativeLinearAccel();
      math::Vector3 worldLinearAccel =
        ballNoFrictionModel->GetWorldLinearAccel();

      if (world->GetSimTime().Double() <= 1.0)
      {
        // freefall
        EXPECT_NEAR(imuLinearAccel.x, 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.y, 0, IMU_TOL);
        EXPECT_NEAR(imuLinearAccel.z, 0, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.x, g.x, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.y, g.y, IMU_TOL);
        EXPECT_NEAR(relativeLinearAccel.z, g.z, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.x, g.x, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.y, g.y, IMU_TOL);
        EXPECT_NEAR(worldLinearAccel.z, g.z, IMU_TOL);
      }
      else if (world->GetSimTime().Double() >= 1.3 &&
               world->GetSimTime().Double() <= 1.751)
      {
        // on the ramp
        const double rampAngle = 0.5;
        double gMag = g.GetLength();
        double imuMag = imuLinearAccel.GetLength();
        EXPECT_NEAR(imuMag, gMag*cos(rampAngle), IMU_TOL);

        double relMag = relativeLinearAccel.GetLength();
        EXPECT_NEAR(relMag, gMag*sin(rampAngle), IMU_TOL);
        double worMag = worldLinearAccel.GetLength();
        EXPECT_NEAR(worMag, gMag*sin(rampAngle), IMU_TOL);
      }
      else if (world->GetSimTime().Double() >= 1.8)
      {
        // on the ground
        double imuMag = imuLinearAccel.GetLength();
        double gMag = g.GetLength();
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
  math::Quaternion imuOrientation = ballFloatingImu->GetOrientation();
  // note this test fails without normailzation because
  // quaternion returned is unnormalized(0, 0, 0, 0),
  // not default value of (1, 0, 0, 0)
  imuOrientation.Normalize();
  EXPECT_NEAR(1, imuOrientation.w, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation.x, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation.y, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation.z, IMU_TOL);

  // test that SetReferenceOrientation sets orientation to argument
  // in this test case, assume world is NWU (X-North, Y-West, Z-Up),
  // then transform from NWU to NED is below:
  math::Pose nwuToNEDReference =
    math::Pose(math::Vector3(0, 0, 0),
    math::Quaternion(M_PI, 0, 0));
  // declare NED frame the reference frame for this IMU
  ballFloatingImu->SetWorldToReferencePose(nwuToNEDReference);

  // let messages propagate asynchronously
  world->Step(1000);

  // orientation of the imu in NED frame
  imuOrientation = ballFloatingImu->GetOrientation();

  EXPECT_NEAR(imuOrientation.w, 0, IMU_TOL);
  EXPECT_NEAR(imuOrientation.x, -1, IMU_TOL);
  EXPECT_NEAR(imuOrientation.y, 0, IMU_TOL);
  EXPECT_NEAR(imuOrientation.z, 0, IMU_TOL);

  // imu orientation in world frame
  math::Quaternion imuWorldOrientation =
    imuOrientation * nwuToNEDReference.rot;

  EXPECT_NEAR(imuWorldOrientation.w, 1, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.x, 0, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.y, 0, IMU_TOL);
  EXPECT_NEAR(imuWorldOrientation.z, 0, IMU_TOL);

  // floating ball 2
  // This "robot" starts with a yaw of 1.8 rad from world frame.
  // test that SetReferencePose resets orientation to identity
  ballFloatingImu2->SetReferencePose();
  math::Quaternion imuOrientation2 = ballFloatingImu2->GetOrientation();
  // note this test fails without normailzation because
  // quaternion returned is unnormalized(0, 0, 0, 0),
  // not default value of (1, 0, 0, 0)
  imuOrientation2.Normalize();
  EXPECT_NEAR(1, imuOrientation2.w, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation2.x, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation2.y, IMU_TOL);
  EXPECT_NEAR(0, imuOrientation2.z, IMU_TOL);

  // test that SetReferenceOrientation sets orientation to argument
  // in this test case, assume world is NWU (X-North, Y-West, Z-Up),
  // then transform from NWU to NED is below:
  math::Pose nwuToNEDReference2 =
    math::Pose(math::Vector3(0, 0, 0),
    math::Quaternion(M_PI, 0, 0));
  // declare NED frame the reference frame for this IMU
  ballFloatingImu2->SetWorldToReferencePose(nwuToNEDReference2);

  // let messages propagate asynchronously
  world->Step(1000);

  // orientation of the imu in NED frame
  const double imu2Angle = 1.8;
  imuOrientation2 = ballFloatingImu2->GetOrientation();
  math::Vector3 rpy2 = imuOrientation2.GetAsEuler();
  EXPECT_NEAR(fabs(rpy2.x), M_PI, IMU_TOL);
  EXPECT_NEAR(rpy2.y, 0, IMU_TOL);
  EXPECT_NEAR(rpy2.z, -imu2Angle, IMU_TOL);

  // imu orientation in world frame
  math::Quaternion imuWorldOrientation2 =
    imuOrientation2 * nwuToNEDReference2.rot;
  math::Vector3 rpyWorld2 = imuWorldOrientation2.GetAsEuler();
  EXPECT_NEAR(rpyWorld2.x, 0, IMU_TOL);
  EXPECT_NEAR(rpyWorld2.y, 0, IMU_TOL);
  EXPECT_NEAR(rpyWorld2.z, -imu2Angle, IMU_TOL);

  // turn floating ball 2 by -1.8 rad yaw, and see if two floating
  // balls orientation match
  ballFloatingLink2->SetWorldPose(math::Pose(3.0, -3.40, 0.95, 0.0, 0.0, 0.0));

  // let messages propagate asynchronously
  world->Step(1000);

  // get orientation of two floating balls and compare them
  imuOrientation = ballFloatingImu->GetOrientation();
  math::Vector3 rpy = imuOrientation.GetAsEuler();

  imuOrientation2 = ballFloatingImu2->GetOrientation();
  rpy2 = imuOrientation2.GetAsEuler();

  EXPECT_NEAR(fabs(rpy.x), fabs(rpy2.x), IMU_TOL);
  EXPECT_NEAR(rpy.y, rpy2.y, IMU_TOL);
  EXPECT_NEAR(rpy.z, rpy2.z, IMU_TOL);

  // turn floating ball 2 by 0.6 rad yaw, apply torque about imu local Y-axis
  // and test AngularVelocity is expressed in local frame.
  const double yaw = 0.6;
  ballFloatingLink2->SetWorldPose(math::Pose(3.0, -3.40, 0.95, 0.0, 0.0, yaw));
  world->Step(100);

  ballFloatingLink2->AddRelativeTorque(math::Vector3(0, 150.0, 0));

  // expected velocity calculation
  const double iyy = 0.1;  // kg*m^2
  const double tau = 150.0;  // Nm
  const double dt = 0.001;  // sec
  const double pDot = tau / iyy * dt;  // 1.5 m/s  // pitch rate
  const int nsteps = 1000;
  const double p = pDot * (nsteps-1) * dt;

  // let messages propagate asynchronously
  world->Step(nsteps);

  // get orientation of two floating balls and compare them
  imuOrientation2 = ballFloatingImu2->GetOrientation();
  rpy2 = imuOrientation2.GetAsEuler();
  math::Vector3 rpyDot2 = ballFloatingImu2->GetAngularVelocity();

  // grab from running test, but we should be able to compute this too
  EXPECT_NEAR(fabs(rpy2.x), M_PI, IMU_TOL);  // because NED
  EXPECT_NEAR(rpy2.y, -p, IMU_TOL);
  EXPECT_NEAR(rpy2.z, -yaw, IMU_TOL);

  EXPECT_NEAR(rpyDot2.x, 0, IMU_TOL);
  EXPECT_NEAR(rpyDot2.y, pDot, IMU_TOL);
  EXPECT_NEAR(rpyDot2.z, 0, IMU_TOL);

  // turn floating ball 2 by 0.5 rad pitch, apply force about
  // positive world z-axis
  // and test if LinearAcceleration is expressed in local frame.
  const double pit = 0.5;
  ballFloatingImu2->SetWorldToReferencePose(math::Pose());
  ballFloatingLink2->Reset();
  ballFloatingLink2->SetWorldPose(math::Pose(3.0, -3.40, 0.95, 0.0, pit, 0.0));
  world->Step(100);
  for (int i = 0; i < 1000; ++i)
  {
    const double f = 13.8;
    ballFloatingLink2->AddForce(math::Vector3(0, 0, f));
    world->Step(1);
    const double m = 5.0;
    const double a = f / m;
    math::Vector3 linAcc2 = ballFloatingImu2->GetLinearAcceleration();
    this->GetGravity(ballFloatingImu2->GetOrientation(), g);
    if (i > 100)
    {
      // THERE MUST BE A BETTER WAY TO DO THIS...
      // need to take 100 stesps to ensure that
      // imu readings are passed through from asynchronous transport
      EXPECT_NEAR(linAcc2.x, -a*sin(pit) - g.x, IMU_TOL);
      EXPECT_NEAR(linAcc2.y, 0 - g.y, IMU_TOL);
      EXPECT_NEAR(linAcc2.z, a*cos(pit) - g.z, IMU_TOL);
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0.5, -1.0, 0.2));

  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler());

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1, rateMean, accelMean, orientation);

  EXPECT_NEAR(rateMean.x, 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.y, 0.0, IMU_TOL);
  EXPECT_NEAR(rateMean.z, 0.0, IMU_TOL);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  EXPECT_NEAR(accelMean.x, -g.x, IMU_TOL);
  EXPECT_NEAR(accelMean.y, -g.y, IMU_TOL);
  EXPECT_NEAR(accelMean.z, -g.z, IMU_TOL);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(0.3, -1.4, 2.0));

  double rateNoiseMean = 1.0;
  double rateNoiseStddev = 0.1;
  double rateBiasMean = 0.0;
  double rateBiasStddev = 0.0;
  double accelNoiseMean = -10.0;
  double accelNoiseStddev = 0.1;
  double accelBiasMean = 0.0;
  double accelBiasStddev = 0.0;
  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.x - (accelNoiseMean + accelBiasMean) + g.x);
  d2 = fabs(accelMean.x - (accelNoiseMean - accelBiasMean) + g.x);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.y - (accelNoiseMean + accelBiasMean) + g.y);
  d2 = fabs(accelMean.y - (accelNoiseMean - accelBiasMean) + g.y);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.z - (accelNoiseMean + accelBiasMean) + g.z);
  d2 = fabs(accelMean.z - (accelNoiseMean - accelBiasMean) + g.z);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
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
  math::Pose testPose(math::Vector3(0, 0, 0.05),
      math::Quaternion(-0.3, 0.5, 1.0));

  double rateNoiseMean = 0.0;
  double rateNoiseStddev = 0.0;
  double rateBiasMean = 1.0;
  double rateBiasStddev = 0.1;
  double accelNoiseMean = 0.0;
  double accelNoiseStddev = 0.0;
  double accelBiasMean = 5.0;
  double accelBiasStddev = 0.1;
  SpawnImuSensor(modelName, imuSensorName, testPose.pos,
      testPose.rot.GetAsEuler(), "gaussian",
      rateNoiseMean, rateNoiseStddev,
      rateBiasMean, rateBiasStddev,
      accelNoiseMean, accelNoiseStddev,
      accelBiasMean, accelBiasStddev);

  sensors::ImuSensorPtr imu =
    boost::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  ASSERT_TRUE(imu);
  imu->Init();
  math::Vector3 rateMean, accelMean;
  math::Quaternion orientation;
  this->GetImuData(imu, 1000, rateMean, accelMean, orientation);

  double d1, d2;
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(rateMean.x - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.x - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.y - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.y - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);
  d1 = fabs(rateMean.z - (rateNoiseMean + rateBiasMean));
  d2 = fabs(rateMean.z - (rateNoiseMean - rateBiasMean));
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*rateNoiseStddev + 3*rateBiasStddev);

  math::Vector3 g;
  this->GetGravity(testPose.rot, g);
  // Have to account for the fact that the bias might be sampled as positive
  // or negative
  d1 = fabs(accelMean.x - (accelNoiseMean + accelBiasMean) + g.x);
  d2 = fabs(accelMean.x - (accelNoiseMean - accelBiasMean) + g.x);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.y - (accelNoiseMean + accelBiasMean) + g.y);
  d2 = fabs(accelMean.y - (accelNoiseMean - accelBiasMean) + g.y);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);
  d1 = fabs(accelMean.z - (accelNoiseMean + accelBiasMean) + g.z);
  d2 = fabs(accelMean.z - (accelNoiseMean - accelBiasMean) + g.z);
  EXPECT_NEAR(0.0, std::min(d1, d2),
              3*accelNoiseStddev + 3*accelBiasStddev);

  // Orientation should be identity, since it is reported relative
  // to reference pose.
  EXPECT_NEAR(orientation.x, 0, IMU_TOL);
  EXPECT_NEAR(orientation.y, 0, IMU_TOL);
  EXPECT_NEAR(orientation.z, 0, IMU_TOL);
  EXPECT_NEAR(orientation.w, 1, IMU_TOL);
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
  math::Rand::SetSeed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
