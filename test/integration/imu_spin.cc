/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/sensors/ImuSensor.hh"
#include "test/ServerFixture.hh"
#include "test/integration/helper_physics_generator.hh"

// numerical tolerance
const double g_imu_tol = 1e-5;

using namespace gazebo;
class ImuSpinTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  /// \brief Test world with many spheres spinning with no friction.
  /// Each sphere has an IMU at its center, with the IMU Z axis
  /// pointing up. Each sphere has a different link frame.
  /// Verify that acceleration reads [0, 0, 9.8] for each IMU.
  public: void SpinTestWorld(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void ImuSpinTest::SpinTestWorld(const std::string &_physicsEngine)
{
  Load("worlds/imu_ball_spin.world", true, _physicsEngine);

  // get world
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // get gravity
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != nullptr);
  auto gravity = physics->GetGravity();
  EXPECT_EQ(gravity, math::Vector3(0, 0, -9.8));

  // get sensor manager
  auto sensorManager = sensors::SensorManager::Instance();
  ASSERT_TRUE(sensorManager != nullptr);

  auto sensors = sensorManager->GetSensors();
  EXPECT_EQ(sensors.size(), 16);

  // verify sensor name prefix and type
  for (auto sensor : sensors)
  {
    EXPECT_EQ(sensor->GetName().find("sphere_"), 0);
    EXPECT_EQ(sensor->GetType().compare("imu"), 0);
  }

  // step forward in time
  world->Step(100);
  const unsigned int steps = 3000;
  for (unsigned int step = 0; step < steps; ++step)
  {
    world->Step(1);
    for (auto sensor : sensors)
    {
      auto imu = boost::static_pointer_cast<sensors::ImuSensor>(sensor);
      ASSERT_NE(imu, nullptr);
      EXPECT_EQ(sensor->GetLinearAcceleration(), -gravity);
    }
  }
}

TEST_P(ImuSpinTest, ImuSensorTestWorld)
{
  ImuSensorTestWorld(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuSpinTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
