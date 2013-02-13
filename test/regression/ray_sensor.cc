/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "physics/physics.hh"
#include "sensors/sensors.hh"
#include "common/common.hh"
#include "sensors/RaySensor.hh"

#define PHYSICS_TOL 1e-2

using namespace gazebo;

class RaySensor : public ServerFixture
{
  public: void EmptyWorld(const std::string &_physicsEngine);
  public: void RayUnitCylinder(const std::string &_physicsEngine);
};


/*
SpawnRaySensor(const std::string &_modelName,
                 const std::string &_raySensorName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 double _hMinAngle = -2.0, double _hMaxAngle = 2.0,
                 double _minRange = 0.08, double _maxRange = 10,
                 double _rangeResolution = 0.01, unsigned int _samples = 640)
                 */

void RaySensor::EmptyWorld(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  math::Pose testPose(math::Vector3(0, 0, 0),
      math::Quaternion(0, 0, 0));

  double hMinAngle = -2.0;
  double hMaxAngle = 2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(sensor);

  EXPECT_NEAR(raySensor->GetAngleMin().Radian(), -2.0, PHYSICS_TOL);
  EXPECT_NEAR(raySensor->GetAngleMax().Radian(), 2.0, PHYSICS_TOL);
  EXPECT_NEAR(raySensor->GetRangeMin(), 0.1, PHYSICS_TOL);
  EXPECT_NEAR(raySensor->GetRangeMax(), 5.0, PHYSICS_TOL);
  EXPECT_NEAR(raySensor->GetRangeResolution(), 0.02, PHYSICS_TOL);
  EXPECT_EQ(raySensor->GetRayCount(), 320);
}

TEST_F(RaySensor, EmptyWorldODE)
{
  EmptyWorld("ode");
}

TEST_F(RaySensor, EmptyWorldBullet)
{
  EmptyWorld("bullet");
}

void RaySensor::RayUnitCylinder(const std::string &_physicsEngine)
{
  // Test ray sensor with 3 cylinders in the world.
  // First place 2 of 3 cylinders within range and verify range values,
  // then move all 3 cylinder out of range and verify range values

  Load("worlds/empty.world", true, _physicsEngine);

  std::string modelName = "ray_model";
  std::string raySensorName = "ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  math::Pose testPose(math::Vector3(0, 0, 0),
      math::Quaternion(0, 0, 0));

  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string cylinder01 = "cylinder_01";
  std::string cylinder02 = "cylinder_02";
  std::string cylinder03 = "cylinder_03";

  // cylinder in front of ray sensor
  math::Pose cylinder01Pose(math::Vector3(1, 0, 0),
      math::Quaternion(0, 0, 0));
  // cylinder on the right of ray sensor
  math::Pose cylinder02Pose(math::Vector3(0, -1, 0),
      math::Quaternion(0, 0, 0));
  // cylinder on the left of the ray sensor but out of range
  math::Pose cylinder03Pose(math::Vector3(0, maxRange + 1, 0),
      math::Quaternion(0, 0, 0));

  SpawnCylinder(cylinder01, cylinder01Pose.pos,
      cylinder01Pose.rot.GetAsEuler());

  SpawnCylinder(cylinder02, cylinder02Pose.pos,
      cylinder02Pose.rot.GetAsEuler());

  SpawnCylinder(cylinder03, cylinder03Pose.pos,
      cylinder03Pose.rot.GetAsEuler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::shared_dynamic_cast<sensors::RaySensor>(sensor);

  raySensor->Init();
  raySensor->Update(true);

  int mid = samples / 2;
  double unitCylinderRadius = 0.5;
  double expectedRangeAtMidPoint = cylinder01Pose.pos.x - unitCylinderRadius;

  // WARNING: gazebo returns distance to object from min range
  // issue #503
  expectedRangeAtMidPoint -= minRange;

  EXPECT_NEAR(raySensor->GetRange(mid), expectedRangeAtMidPoint, PHYSICS_TOL);
  EXPECT_NEAR(raySensor->GetRange(0), expectedRangeAtMidPoint, PHYSICS_TOL);

  // WARNING: for readings of no return, gazebo returns max range rather
  // than +inf. issue #124
  EXPECT_NEAR(raySensor->GetRange(samples-1), maxRange, PHYSICS_TOL);

  // Move all cylinders out of range
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world !=NULL);

  world->GetModel(cylinder01)->SetWorldPose(
        math::Pose(math::Vector3(maxRange + 1, 0, 0),
        math::Quaternion(0, 0, 0)));
  world->GetModel(cylinder02)->SetWorldPose(
        math::Pose(math::Vector3(0, -(maxRange + 1), 0),
        math::Quaternion(0, 0, 0)));
  world->StepWorld(1);
  raySensor->Update(true);

  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    EXPECT_NEAR(raySensor->GetRange(i), maxRange, PHYSICS_TOL);
  }
}

TEST_F(RaySensor, RayCylindereODE)
{
  RayUnitCylinder("ode");
}

TEST_F(RaySensor, RayCylinderBullet)
{
  RayUnitCylinder("bullet");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
