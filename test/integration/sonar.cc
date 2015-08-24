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
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class SonarTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void GroundPlane(const std::string &_physicsEngine);
  public: void Box(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void SonarTest::GroundPlane(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  sensors::SonarSensorPtr sonar = SpawnSonar("sonar", "sonar",
      ignition::math::Pose3d(0, 0, 1, 0, 0, 0), 0, 2, 0.2);
  ASSERT_TRUE(sonar != NULL);

  physics::ModelPtr model = world->GetModel("sonar");
  ASSERT_TRUE(model != NULL);

  // Sonar should detect the ground plane
  world->Step(100);
  EXPECT_NEAR(sonar->GetRange(), 1.0, 0.01);

  // Rotate the model, and the sonar should not see the ground plane
  model->SetWorldPose(ignition::math::Pose3d(0, 0, 1, 0, 1.5707, 0));
  world->Step(100);
  EXPECT_NEAR(sonar->GetRange(), 2.0, 0.01);
}

/////////////////////////////////////////////////
TEST_P(SonarTest, GroundPlane)
{
  GroundPlane(GetParam());
}

/////////////////////////////////////////////////
void SonarTest::Box(const std::string &_physicsEngine)
{
  Load("worlds/sonar_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  sensors::SonarSensorPtr sonar =
    boost::dynamic_pointer_cast<sensors::SonarSensor>(
        sensors::get_sensor("sonar"));
  ASSERT_TRUE(sonar != NULL);

  physics::ModelPtr model = world->GetModel("sonar_model");
  ASSERT_TRUE(model != NULL);

  // Sonar should detect the ground plane
  world->Step(100);
  EXPECT_NEAR(sonar->GetRange(), 1.5, 0.01);

  // Move the model, and the sonar should not see the ground plane
  model->SetWorldPose(ignition::math::Pose3d(2, 2, 10, 0, 0, 0));
  world->Step(100);
  EXPECT_NEAR(sonar->GetRange(), 2.0, 0.01);
}

/////////////////////////////////////////////////
TEST_P(SonarTest, Box)
{
  Box(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, SonarTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
