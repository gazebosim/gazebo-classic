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
#include <gtest/gtest.h>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/sensors/AltimeterSensor.hh"

#define TOL 1e-4

using namespace gazebo;

/// \brief Test class for the altimeter sensor
class AltimeterSensor_TEST : public ServerFixture,
  public testing::WithParamInterface<const char*>
{
  /// \brief Check that a simple altimeter works correctly
  /// The default should be inserted with altitude = 0 and velocity =  0
  /// \param[in] _physicsEngine The type of physics engine to use.
  public: void BasicAltimeterSensorCheck(const std::string &_physicsEngine);

  /// \brief Check that an alimeter at a non-zero altitude works.
  /// \param[in] _physicsEngine The type of physics engine to use.
  public: void NonzeroAltimeterSensorCheck(const std::string &_physicsEngine);
};

// An altitude sensor
static std::string altSensorString =
"<sdf version='1.5'>"
"  <sensor name='altimeter' type='altimeter'>"
"    <always_on>1</always_on>"
"    <update_rate>10.0</update_rate>"
"    <altimeter>"
"    </altimeter>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
void AltimeterSensor_TEST::BasicAltimeterSensorCheck(
  const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  physics::WorldPtr world = physics::get_world("default");

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(altSensorString, sdf);

  // Create the altimeter sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
    std::string("default::ground_plane::link::altimeter"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the altimeter sensor
  sensors::AltimeterSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::AltimeterSensor>
      (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  // By default the altitude of the sensor should be zero
  EXPECT_DOUBLE_EQ(sensor->VerticalPosition(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->VerticalVelocity(), 0.0);
}

/////////////////////////////////////////////////
// If inserted at X=0,Y=0,Z=10m
void AltimeterSensor_TEST::NonzeroAltimeterSensorCheck(
  const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Spawn an altimeter sensor at a height of 10m
  std::string modelName = "altModel";
  std::string altSensorName = "altSensor";
  math::Pose modelPose(0, 0, 10, 0, 0, 0);
  std::string topic = "~/" + altSensorName + "_" + _physicsEngine;
  SpawnUnitAltimeterSensor(modelName, altSensorName,
      "box", topic, modelPose.pos, modelPose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(altSensorName);
  sensors::AltimeterSensorPtr altSensor =
      boost::dynamic_pointer_cast<sensors::AltimeterSensor>(sensor);

  ASSERT_TRUE(altSensor != NULL);

  sensors::SensorManager::Instance()->Init();
  altSensor->SetActive(true);

  // Check for match
  EXPECT_DOUBLE_EQ(altSensor->VerticalPosition(), 10.0);
  EXPECT_DOUBLE_EQ(altSensor->VerticalVelocity(), 0.0);
}

/////////////////////////////////////////////////
TEST_P(AltimeterSensor_TEST, BasicAltimeterSensorCheck)
{
  BasicAltimeterSensorCheck(GetParam());
}

/////////////////////////////////////////////////
TEST_P(AltimeterSensor_TEST, NonzeroAltimeterSensorCheck)
{
  NonzeroAltimeterSensorCheck(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, AltimeterSensor_TEST,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
