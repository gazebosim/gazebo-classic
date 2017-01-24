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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL_GRAVITY 1e-4
#define TOL_FORCES 1.0
#define TOL_TORQUES 2.0

using namespace gazebo;

class Issue940Test : public ServerFixture,
                     public testing::WithParamInterface<const char*>
{
  public: void ForceTorqueSensorFrameTest(const std::string &_physicsEngine);
  public: void ExpectForceTorqueMeasure(const std::string &_sensorName,
                                    const ignition::math::Vector3d &_expForce,
                                    const ignition::math::Vector3d &_expTorque,
                                    sensors::SensorManager *_mgr);
};

/////////////////////////////////////////////////////////////////////
// \brief Compare force and torque measures with analytical solutions
void Issue940Test::ForceTorqueSensorFrameTest(const std::string &_physicsEngine)
{
  if (SDF_MAJOR_VERSION < 2 ||
      std::string(SDF_VERSION_FULL) == std::string("2.0.0"))
  {
    gzerr << "Skipping test since SDF is too old" << std::endl;
    return;
  }
  if (_physicsEngine == "bullet")
  {
    gzerr << "Skipping this test for " << _physicsEngine
          << " since it has a race condition that randomly gives"
          << " incorrect force-torque readings."
          << std::endl;
    return;
  }
  bool worldPaused = true;
  Load("worlds/force_torque_frame_test.world", worldPaused, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();
  ASSERT_TRUE(mgr != NULL);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Make sure that the sensor are correctly simulated
  world->Step(20);

  // Assume gravity on z axis
  auto grav = world->Gravity();
  EXPECT_NEAR(grav[0], 0, TOL_GRAVITY);
  EXPECT_NEAR(grav[1], 0, TOL_GRAVITY);

  double g = grav[2];

  double m0 = 10.0;
  double m1 = 10.0;

  double mAll = m0 + m1;

  double cog_y_1 = 3.0;
  double cog_y_0 = -3.0;
  double delta_x_joint_12 = 1.0;

  // For details on the expected answers, check force_torque_frame_test.world
  ExpectForceTorqueMeasure("force_torque_01_parent_and_parent_to_child",
    ignition::math::Vector3d(0, 0, -g*mAll),
    ignition::math::Vector3d(-g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_01_parent_and_child_to_parent",
    ignition::math::Vector3d(0, 0, g*mAll),
    ignition::math::Vector3d(g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_01_child_and_parent_to_child",
    ignition::math::Vector3d(0, 0, -g*mAll),
    ignition::math::Vector3d(-g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_01_child_and_child_to_parent",
    ignition::math::Vector3d(0, 0, g*mAll),
    ignition::math::Vector3d(g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_01_sensor_and_parent_to_child",
    ignition::math::Vector3d(0, 0, -g*mAll),
    ignition::math::Vector3d(-g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_01_sensor_and_child_to_parent",
    ignition::math::Vector3d(0, 0, g*mAll),
    ignition::math::Vector3d(g*(m0*cog_y_0+m1*cog_y_1), 0, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_12_parent_and_parent_to_child",
    ignition::math::Vector3d(0, 0, -g*m1),
    ignition::math::Vector3d(-g*m1*cog_y_1, -g*m1*delta_x_joint_12, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_12_parent_and_child_to_parent",
    ignition::math::Vector3d(0, 0, g*m1),
    ignition::math::Vector3d(g*m1*cog_y_1, g*m1*delta_x_joint_12, 0), mgr);
  ExpectForceTorqueMeasure("force_torque_12_child_and_parent_to_child",
    ignition::math::Vector3d(0, -g*m1, 0),
    ignition::math::Vector3d(-g*m1*cog_y_1, 0, g*m1*delta_x_joint_12), mgr);
  ExpectForceTorqueMeasure("force_torque_12_child_and_child_to_parent",
    ignition::math::Vector3d(0, g*m1, 0),
    ignition::math::Vector3d(g*m1*cog_y_1, 0, -g*m1*delta_x_joint_12), mgr);
  ExpectForceTorqueMeasure("force_torque_12_sensor_and_parent_to_child",
    ignition::math::Vector3d(0, -g*m1, 0),
    ignition::math::Vector3d(g*m1*delta_x_joint_12, 0, g*m1*cog_y_1), mgr);
  ExpectForceTorqueMeasure("force_torque_12_sensor_and_child_to_parent",
    ignition::math::Vector3d(0, g*m1, 0),
    ignition::math::Vector3d(-g*m1*delta_x_joint_12, 0, -g*m1*cog_y_1), mgr);
}

////////////////////////////////////////////////////////////////////
// \brief Expect force and torque measures for a force torque sensor
void Issue940Test::ExpectForceTorqueMeasure(const std::string &_sensorName,
                                    const ignition::math::Vector3d &_expForce,
                                    const ignition::math::Vector3d &_expTorque,
                                    sensors::SensorManager *_mgr)
{
  sensors::ForceTorqueSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        _mgr->GetSensor(_sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);

  ignition::math::Vector3d mesForce = sensor->Force();
  ignition::math::Vector3d mesTorque = sensor->Torque();

  gzdbg << "sensorName: " << _sensorName << std::endl;
  gzdbg << "mesForce :  " << mesForce << std::endl;
  gzdbg << "expForce :  " << _expForce << std::endl;
  gzdbg << "mesTorque : " << mesTorque << std::endl;
  gzdbg << "expTorque : " << _expTorque << std::endl;

  EXPECT_NEAR(_expForce.X(), mesForce.X(), TOL_FORCES);
  EXPECT_NEAR(_expForce.Y(), mesForce.Y(), TOL_FORCES);
  EXPECT_NEAR(_expForce.Z(), mesForce.Z(), TOL_FORCES);

  EXPECT_NEAR(_expTorque.X(), mesTorque.X(), TOL_TORQUES);
  EXPECT_NEAR(_expTorque.Y(), mesTorque.Y(), TOL_TORQUES);
  EXPECT_NEAR(_expTorque.Z(), mesTorque.Z(), TOL_TORQUES);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
// \brief Test for issue #940
TEST_P(Issue940Test, ForceTorqueSensorFrameTest)
{
  ForceTorqueSensorFrameTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue940Test, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
