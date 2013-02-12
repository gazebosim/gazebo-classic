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
  SpawnRaySensor(modelName, raySensorName, testPose.pos,
      testPose.rot.GetAsEuler());
}

TEST_F(RaySensor, EmptyWorldODE)
{
  EmptyWorld("ode");
}


TEST_F(RaySensor, EmptyWorldBullet)
{
  EmptyWorld("bullet");
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
