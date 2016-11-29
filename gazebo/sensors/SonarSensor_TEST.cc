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

#include <gtest/gtest.h>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

typedef std::tr1::tuple<const char *, bool> const_char_bool;

class SonarSensor_TEST : public ServerFixture,
                         public ::testing::WithParamInterface<const_char_bool>
{
  /// \brief Test Creation of a Sonar sensor.
  /// \param[in] _physicsEngine Name of physics engine to use.
  /// \param[in] _paused Start paused if true.
  public: void CreateSonar(const std::string &_physicsEngine, bool _paused);

  /// \brief Test the sonar demo world.
  /// \param[in] _physicsEngine Name of physics engine to use.
  /// \param[in] _paused Start paused if true.
  public: void DemoWorld(const std::string &_physicsEngine, bool _paused);

  /// \brief Test sonar with just a ground plane.
  /// \param[in] _physicsEngine Name of physics engine to use.
  public: void GroundPlane(const std::string &_physicsEngine);
};

static std::string sonarSensorString =
"<sdf version='1.4'>"
"  <sensor name='sonar' type='sonar'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <sonar>"
"      <min>0</min>"
"      <max>1</max>"
"      <radius>0.3</radius>"
"    </sonar>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
void SonarSensor_TEST::CreateSonar(const std::string &_physicsEngine,
                                   bool _paused)
{
  Load("worlds/empty.world", _paused, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(sonarSensorString, sdf);

  physics::WorldPtr world = physics::get_world("default");
  physics::ModelPtr model = world->GetModel("ground_plane");
  physics::LinkPtr link = model->GetLink("link");

  // Create the Sonar sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", link->GetId());

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::sonar"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the sonar sensor
  sensors::SonarSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::SonarSensor>(mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_TRUE(sensor != nullptr);

  // Update the sensor
  sensor->Update(true);

  EXPECT_TRUE(sensor->IsActive());

  EXPECT_DOUBLE_EQ(sensor->RangeMin(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->RangeMax(), 1.0);
  EXPECT_DOUBLE_EQ(sensor->Radius(), 0.3);
  EXPECT_DOUBLE_EQ(sensor->Range(), 1.0);

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
void SonarSensor_TEST::DemoWorld(const std::string &_physicsEngine,
                                 bool _paused)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support sonar sensor, "
          << "see issue #2062."
          << std::endl;
    return;
  }

  Load("worlds/sonar_demo.world", _paused, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != nullptr);
  world->Step(100);

  // Sonar sensor name
  std::string sensorName = "sonar";

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the sonar sensor
  sensors::SonarSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::SonarSensor>(mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  ASSERT_TRUE(sensor != nullptr);

  // Update the sensor
  sensor->Update(true);

  EXPECT_TRUE(sensor->IsActive());

  EXPECT_DOUBLE_EQ(sensor->RangeMin(), 0.0);
  EXPECT_DOUBLE_EQ(sensor->RangeMax(), 2.0);
  EXPECT_DOUBLE_EQ(sensor->Radius(), 0.3);
  if (_physicsEngine == "ode")
    EXPECT_NEAR(sensor->Range(), 1.4999, 1e-3);
  else
  {
    gzerr << "Sonar range sensing only works in ODE, issue #1038"
          << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void SonarSensor_TEST::GroundPlane(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Sonar range sensing only works in ODE, issue #1038"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  sensors::SonarSensorPtr sonar = SpawnSonar("sonar", "sonar",
      ignition::math::Pose3d(0, 0, 1, 0, 0, 0), 0, 2, 0.2);
  ASSERT_TRUE(sonar != nullptr);

  physics::ModelPtr model = world->GetModel("sonar");
  ASSERT_TRUE(model != nullptr);

  // Wait for collision engine to turn over
  common::Time::MSleep(1000);

  // Sonar should detect the ground plane
  sonar->Update(true);
  EXPECT_NEAR(sonar->Range(), 1.0, 0.01);

  // Rotate the model, and the sonar should not see the ground plane
  model->SetWorldPose(ignition::math::Pose3d(0, 0, 1, 0, 1.5707, 0));

  // Wait for collision engine to turn over
  common::Time::MSleep(1000);

  sonar->Update(true);
  EXPECT_NEAR(sonar->Range(), 2.0, 0.01);
}

TEST_P(SonarSensor_TEST, CreateSonar)
{
  std::string physics = std::tr1::get<0>(GetParam());
  bool paused = std::tr1::get<1>(GetParam());
  gzdbg << "Physics " << physics
        << " paused " << paused
        << std::endl;
  CreateSonar(physics, paused);
}

TEST_P(SonarSensor_TEST, DemoWorld)
{
  std::string physics = std::tr1::get<0>(GetParam());
  bool paused = std::tr1::get<1>(GetParam());
  gzdbg << "Physics " << physics
        << " paused " << paused
        << std::endl;
  DemoWorld(physics, paused);
}

TEST_P(SonarSensor_TEST, GroundPlane)
{
  std::string physics = std::tr1::get<0>(GetParam());
  GroundPlane(physics);
}

INSTANTIATE_TEST_CASE_P(SonarTests, SonarSensor_TEST,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values(false, true)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
