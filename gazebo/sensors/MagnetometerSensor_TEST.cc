/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include <ignition/math/Vector3.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/sensors/MagnetometerSensor.hh"

#define TOL 1e-4

using namespace gazebo;

class MagnetometerSensor_TEST : public ServerFixture,
                                public testing::WithParamInterface<const char*>
{
  /// \brief Check that a model at (0,0,0,0,0,0) has mag field equal to global
  /// \param[in] _physicsEngine The type of physics engine to use.
  public: void BasicMagnetometerSensorCheck(const std::string &_physicsEngine);

  /// \brief Rotate sensor and check magnetic field.
  /// \param[in] _physicsEngine The type of physics engine to use.
  public: void RotateMagnetometerSensorCheck(const std::string &_physicsEngine);
};

// A noise-free magnetic field strength sensor
static std::string magSensorString =
"<sdf version='1.5'>"
"  <sensor name='magnetometer' type='magnetometer'>"
"    <always_on>1</always_on>"
"    <update_rate>10.0</update_rate>"
"    <magnetometer>"
"    </magnetometer>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
void MagnetometerSensor_TEST::BasicMagnetometerSensorCheck(
  const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  ASSERT_TRUE(mgr != nullptr);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(magSensorString, sdf);

  // Create the magnetometer sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
    std::string("default::ground_plane::link::magnetometer"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the magnetometer sensor
  sensors::MagnetometerSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::MagnetometerSensor>
      (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // At pose [0,0,0,0,0,0] the body frame magnetic field should be default
  EXPECT_EQ(sensor->MagneticField(), world->MagneticField());
}

/////////////////////////////////////////////////
void MagnetometerSensor_TEST::RotateMagnetometerSensorCheck(
  const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Spawn a magnetometer sensor with a PI/2 aniclockwise rotation about Z axis
  std::string modelName = "magModel";
  std::string magSensorName = "magSensor";
  ignition::math::Pose3d modelPose(0, 0, 0, 0, 0, IGN_PI_2);
  std::string topic = "~/" + magSensorName + "_" + _physicsEngine;
  SpawnUnitMagnetometerSensor(modelName, magSensorName,
      "box", topic, modelPose.Pos(), modelPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(magSensorName);
  sensors::MagnetometerSensorPtr magSensor =
      std::dynamic_pointer_cast<sensors::MagnetometerSensor>(sensor);

  ASSERT_TRUE(magSensor != nullptr);

  sensors::SensorManager::Instance()->Init();
  magSensor->SetActive(true);

  world->Step(10);

  // Determine the magnetic field in the body frame
  ignition::math::Vector3d field = modelPose.Rot().Inverse().RotateVector(
        world->MagneticField());

  // Check for match
  EXPECT_EQ(magSensor->MagneticField(), field);
}

/////////////////////////////////////////////////
TEST_P(MagnetometerSensor_TEST, BasicMagnetometerSensorCheck)
{
  BasicMagnetometerSensorCheck(GetParam());
}

/////////////////////////////////////////////////
TEST_P(MagnetometerSensor_TEST, RotateMagnetometerSensorCheck)
{
  RotateMagnetometerSensorCheck(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, MagnetometerSensor_TEST,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
