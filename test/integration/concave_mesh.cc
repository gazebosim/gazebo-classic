/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/test/helper_physics_generator.hh"

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
    std::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  // No ray should interect a collision.
  for (int i = 0; i < raySensor->RangeCount(); ++i)
    EXPECT_DOUBLE_EQ(raySensor->Range(i), IGN_DBL_INF);
}

/////////////////////////////////////////////////
void ConcaveMeshTest::SubmeshCollisionTest(const std::string &_physicsEngine)
{
  Load("worlds/concave_submesh_collision_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  world->Step(100);

  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  EXPECT_DOUBLE_EQ(raySensor->Range(0), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(1), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(2), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(3), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(4), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(5), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(6), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(7), IGN_DBL_INF);
  EXPECT_NEAR(raySensor->Range(8), 1.92439, 1e-2);
  EXPECT_NEAR(raySensor->Range(9), 1.86443, 1e-2);
  EXPECT_NEAR(raySensor->Range(10), 1.86443, 1e-2);
  EXPECT_NEAR(raySensor->Range(11), 1.92439, 1e-2);
  EXPECT_DOUBLE_EQ(raySensor->Range(12), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(13), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(14), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(15), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(16), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(17), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(18), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(19), IGN_DBL_INF);
}

/////////////////////////////////////////////////
void ConcaveMeshTest::RayTest(const std::string &_physicsEngine)
{
  Load("worlds/concave_mesh_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  world->Step(100);

  sensors::RaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::RaySensor>(
    sensors::get_sensor("default::hokuyo::link::laser"));
  EXPECT_TRUE(raySensor != NULL);

  EXPECT_DOUBLE_EQ(raySensor->Range(0), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(1), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(2), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(3), IGN_DBL_INF);
  EXPECT_NEAR(raySensor->Range(4), 0.972282, 1e-2);
  EXPECT_NEAR(raySensor->Range(5), 0.967148, 1e-2);
  EXPECT_NEAR(raySensor->Range(6), 0.962889, 1e-2);
  EXPECT_NEAR(raySensor->Range(7), 0.9597, 1e-2);
  EXPECT_NEAR(raySensor->Range(8), 0.957242, 1e-2);
  EXPECT_NEAR(raySensor->Range(9), 0.955562, 1e-2);
  EXPECT_NEAR(raySensor->Range(10), 0.948761, 1e-2);
  EXPECT_NEAR(raySensor->Range(11), 0.847463, 1e-2);
  EXPECT_NEAR(raySensor->Range(12), 0.847665, 1e-2);
  EXPECT_DOUBLE_EQ(raySensor->Range(13), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(14), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(15), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(16), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(17), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(18), IGN_DBL_INF);
  EXPECT_DOUBLE_EQ(raySensor->Range(19), IGN_DBL_INF);
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
