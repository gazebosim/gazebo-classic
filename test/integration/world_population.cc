/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <string>
#include <vector>
#include "helper_physics_generator.hh"
#include "ServerFixture.hh"
#include "gazebo/math/Vector3.hh"

using namespace gazebo;
class WorldEnvPopulationTest : public ServerFixture,
                               public testing::WithParamInterface<const char*>
{
  public: void LoadEnvironment(const std::string &_physicsType);
};

////////////////////////////////////////////////////////////////////////
// LoadCuboidRandom: Verify that the number of elements populated is correct
// and the objects are distributed as expected.
////////////////////////////////////////////////////////////////////////
void WorldEnvPopulationTest::LoadEnvironment(const std::string &_physicsType)
{
  Load("worlds/population.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  // We should have 56 cloned models + the ground plane.
  EXPECT_EQ(world->GetModelCount(), 64 + 1);

  // Check elements distributed as a grid.
  double tolerance = 0.25;
  math::Vector3 initialPos(-0.25, -0.5, 0);
  math::Vector3 expectedPos(initialPos);
  math::Vector3 step(0.25, 0.25, 0);

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      std::string name = std::string("can2_clone_" +
        boost::lexical_cast<std::string>(i * 3 + j));
      math::Vector3 pos = world->GetModel(name)->GetWorldPose().pos;
      EXPECT_NEAR(pos.Distance(expectedPos), 0.0, tolerance);

      expectedPos.x += step.x;
    }
    expectedPos.x = initialPos.x;
    expectedPos.y += step.y;
  }

  // Check that the objects are within the expected cuboid.
  std::vector<physics::ModelPtr> models = world->GetModels();
  for (size_t i = 0; i < models.size(); ++i)
  {
    physics::ModelPtr model = models[i];
    ASSERT_TRUE(model != NULL);

    // This is not a cloned object, skip it.
    if (model->GetName().find("can1_clone") != std::string::npos)
    {
      math::Box boundingBox(math::Vector3(3, 3, 0), math::Vector3(4, 4, 0.1));
      EXPECT_GE(model->GetWorldPose().pos.x, 3 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.y, 3 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.z, 0 - tolerance);
      EXPECT_LE(model->GetWorldPose().pos.x, 4 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.y, 4 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.z, 0.1 + tolerance);
    }
    else if (model->GetName().find("can3_clone") != std::string::npos)
    {
      math::Vector3 centre(-3, 3, 0);
      double radius = 1.0;
      EXPECT_LE(model->GetWorldPose().pos.Distance(centre), radius + tolerance);
    }
    else if (model->GetName().find("can4_clone") != std::string::npos)
    {
      math::Box boundingBox(math::Vector3(1, -4, 0), math::Vector3(5, -2, .01));
      EXPECT_GE(model->GetWorldPose().pos.x, 1 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.y, -4 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.z, 0 - tolerance);
      EXPECT_LE(model->GetWorldPose().pos.x, 5 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.y, -2 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.z, 0.01 + tolerance);
    }
    else if (model->GetName().find("can5_clone") != std::string::npos)
    {
      math::Vector3 centre(-3, -3, 0);
      double radius = 1.0;
      EXPECT_LE(model->GetWorldPose().pos.Distance(centre), radius + tolerance);
    }
    else if (model->GetName().find("can6_clone") != std::string::npos)
    {
      math::Box boundingBox(math::Vector3(-1, 4.5, 0),
                            math::Vector3(1, 5.5, .01));
      EXPECT_GE(model->GetWorldPose().pos.x, -1 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.z, 0 - tolerance);
      EXPECT_LE(model->GetWorldPose().pos.x, 1 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.z, 0.01 + tolerance);
      EXPECT_NEAR(model->GetWorldPose().pos.y, 5.0, tolerance);
    }
    else if (model->GetName().find("can7_clone") != std::string::npos)
    {
      math::Box boundingBox(math::Vector3(-5.5, -1, 0),
                            math::Vector3(-4.5, 1, .01));
      EXPECT_GE(model->GetWorldPose().pos.y, -1 - tolerance);
      EXPECT_GE(model->GetWorldPose().pos.z, 0 - tolerance);
      EXPECT_LE(model->GetWorldPose().pos.y, 1 + tolerance);
      EXPECT_LE(model->GetWorldPose().pos.z, 0.01 + tolerance);
      EXPECT_NEAR(model->GetWorldPose().pos.x, -5.0, tolerance);
    }
    else if (model->GetName().find("can8_clone") != std::string::npos)
    {
      math::Box boundingBox(math::Vector3(5.4, 0.9, 0),
                            math::Vector3(5.6, 1.1, 1.4));
      EXPECT_GE(model->GetWorldPose().pos.z, 0 - tolerance);
      EXPECT_LE(model->GetWorldPose().pos.z, 1.4 + tolerance);
      EXPECT_NEAR(model->GetWorldPose().pos.x, 5.5, tolerance);
      EXPECT_NEAR(model->GetWorldPose().pos.y, 1, tolerance);
    }
  }
}

TEST_P(WorldEnvPopulationTest, LoadEnvironment)
{
  LoadEnvironment(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldEnvPopulationTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
