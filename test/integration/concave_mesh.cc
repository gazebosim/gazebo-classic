/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

class ConcaveMeshTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void RayTest(const std::string &_physicsEngine);
  public: void SubmeshNoCollisionTest(const std::string &_physicsEngine);
  public: void SubmeshCollisionTest(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void ConcaveMeshTest::SubmeshNoCollisionTest(const std::string &_physicsEngine)
{
  Load("worlds/concave_submesh_no_collision_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  world->Step(100);

  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  // No ray should interect a collision.
  for (int i = 0; i < raySensor->GetRangeCount(); ++i)
    EXPECT_NEAR(raySensor->GetRange(i), raySensor->GetRangeMax(), 1e-2);
}

/////////////////////////////////////////////////
void ConcaveMeshTest::SubmeshCollisionTest(const std::string &_physicsEngine)
{
  Load("worlds/concave_submesh_collision_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  world->Step(100);

  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  EXPECT_NEAR(raySensor->GetRange(0), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(1), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(2), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(3), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(4), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(5), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(6), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(7), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(8), 1.92439, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(9), 1.86443, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(10), 1.86443, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(11), 1.92439, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(12), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(13), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(14), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(15), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(16), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(17), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(18), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(19), 10, 1e-2);
}

/////////////////////////////////////////////////
void ConcaveMeshTest::RayTest(const std::string &_physicsEngine)
{
  Load("worlds/concave_mesh_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  world->Step(100);

  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  EXPECT_NEAR(raySensor->GetRange(0), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(1), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(2), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(3), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(4), 0.972282, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(5), 0.967148, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(6), 0.962889, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(7), 0.9597, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(8), 0.957242, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(9), 0.955562, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(10), 0.948761, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(11), 0.847463, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(12), 0.847665, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(13), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(14), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(15), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(16), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(17), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(18), 10, 1e-2);
  EXPECT_NEAR(raySensor->GetRange(19), 10, 1e-2);
}

/////////////////////////////////////////////////
TEST_P(ConcaveMeshTest, SubmeshCollisionTest)
{
  if (std::string(GetParam()) == "simbody" ||
      std::string(GetParam()) == "dart")
  {
    gzdbg << "ConcaveMeshes not supported in " << GetParam() << "\n";
  }
  else
  {
    SubmeshCollisionTest(GetParam());
  }
}

/////////////////////////////////////////////////
TEST_P(ConcaveMeshTest, SubmeshNoCollisionTest)
{
  if (std::string(GetParam()) == "simbody" ||
      std::string(GetParam()) == "dart")
  {
    gzdbg << "ConcaveMeshes not supported in " << GetParam() << "\n";
  }
  else
  {
    SubmeshNoCollisionTest(GetParam());
  }
}

/////////////////////////////////////////////////
TEST_P(ConcaveMeshTest, RayTest)
{
  if (std::string(GetParam()) == "simbody" ||
      std::string(GetParam()) == "dart")
  {
    gzdbg << "ConcaveMeshes not supported in " << GetParam() << "\n";
  }
  else
  {
    RayTest(GetParam());
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ConcaveMeshTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
