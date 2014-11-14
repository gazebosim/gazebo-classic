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

#include <sys/time.h>
#include <gtest/gtest.h>

#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"
#include "gazebo/sensors/ImuSensor.hh"

#define TOL 1e-4

using namespace gazebo;
class ImuSensor_TEST : public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  public: void BasicImuSensorCheck(const std::string &_physicsEngine);
  public: void LinearAccelerationTest(const std::string &_physicsEngine);
};

static std::string imuSensorString =
"<sdf version='1.3'>"
"  <sensor name='imu' type='imu'>"
"    <always_on>1</always_on>"
"    <update_rate>20.000000</update_rate>"
"    <imu>"
"      <topic>/test_imu</topic>"
"    </imu>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
void ImuSensor_TEST::BasicImuSensorCheck(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(imuSensorString, sdf);

  // Create the IMU sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::imu"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the IMU sensor
  sensors::ImuSensorPtr sensor = boost::dynamic_pointer_cast<sensors::ImuSensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  EXPECT_EQ(sensor->GetAngularVelocity(), math::Vector3::Zero);
  EXPECT_EQ(sensor->GetLinearAcceleration(), math::Vector3::Zero);
  EXPECT_EQ(sensor->GetOrientation(), math::Quaternion(0, 0, 0, 0));
}

/////////////////////////////////////////////////
// Drop a model with imu sensor and measure its linear acceleration
void ImuSensor_TEST::LinearAccelerationTest(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double z = 3;
  double gravityZ = physics->GetGravity().z;
  double stepSize = physics->GetMaxStepSize();

  std::string modelName = "imuModel";
  std::string imuSensorName = "imuSensor";
  math::Pose modelPose(0, 0, z, 0, 0, 0);

  std::string topic = "~/" + imuSensorName + "_" + _physicsEngine;
  // spawn imu sensor
  SpawnUnitImuSensor(modelName, imuSensorName,
      "box", topic, modelPose.pos, modelPose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(imuSensorName);
  sensors::ImuSensorPtr imuSensor =
      boost::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

  ASSERT_TRUE(imuSensor != NULL);

  sensors::SensorManager::Instance()->Init();
  imuSensor->SetActive(true);

  EXPECT_EQ(imuSensor->GetAngularVelocity(), math::Vector3::Zero);
  EXPECT_EQ(imuSensor->GetLinearAcceleration(), math::Vector3::Zero);
  EXPECT_EQ(imuSensor->GetOrientation(), math::Quaternion(0, 0, 0, 0));

  // step world and verify imu's linear acceleration is zero on free fall
  world->Step(200);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().x, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().y, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().z, 0, TOL);
  world->Step(1);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().x, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().y, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().z, 0, TOL);

  // Predict time of contact with ground plane.
  double tHit = sqrt((z-0.5) / (-gravityZ));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->GetSimTime().Double();
  double steps = ceil(dtHit / stepSize);
  EXPECT_GT(steps, 0);
  world->Step(steps);

  EXPECT_NEAR(imuSensor->GetLinearAcceleration().x, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().y, 0, TOL);
  EXPECT_NEAR(imuSensor->GetLinearAcceleration().z, -gravityZ, 0.4);
}

/////////////////////////////////////////////////
TEST_P(ImuSensor_TEST, BasicImuSensorCheck)
{
  BasicImuSensorCheck(GetParam());
}

/////////////////////////////////////////////////
TEST_P(ImuSensor_TEST, LinearAccelerationTest)
{
  LinearAccelerationTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuSensor_TEST,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
