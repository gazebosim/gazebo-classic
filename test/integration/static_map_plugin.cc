/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

using namespace gazebo;
class StaticMapTest : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test StaticMapPlugin and verify model sdf and resource
/// files have been successfully created.
TEST_F(StaticMapTest, StaticMapPlugin)
{
  // temporary back up files for testing if cache exists
  std::string modelName = "map_satellite_37.386491_-122.065255_100_100";
  std::string basePath = common::SystemPaths::Instance()->GetLogPath();
  std::string modelPath = basePath + "/models/" + modelName;
  std::string modelBackupPath = modelPath + "_bk";

  if (common::exists(modelPath))
  {
    boost::filesystem::remove_all(modelBackupPath);
    boost::filesystem::rename(modelPath, modelBackupPath);
    EXPECT_TRUE(common::exists(modelBackupPath));
  }

  // there should be no more model cache
  EXPECT_FALSE(common::exists(modelPath));

  // Test plugin for creating textured map model
  this->Load("worlds/static_map_plugin.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // check that map model is spawned into the world by plugin
  WaitUntilEntitySpawn(modelName, 300, 300);
  physics::ModelPtr mapModel = world->ModelByName(modelName);
  ASSERT_TRUE(mapModel != nullptr);

  // verify basic model properties
  EXPECT_TRUE(mapModel->IsStatic());
  EXPECT_EQ(mapModel->WorldPose(), ignition::math::Pose3d::Zero);

  // links
  auto links = mapModel->GetLinks();
  EXPECT_FALSE(links.empty());
  auto link = links[0];
  ASSERT_TRUE(link != nullptr);

  // collisions
  auto collisions = link->GetCollisions();
  EXPECT_FALSE(collisions.empty());

  // verify model dir structure and files
  EXPECT_TRUE(common::exists(modelPath));
  EXPECT_TRUE(common::isFile(modelPath + "/model.sdf"));
  EXPECT_TRUE(common::isFile(modelPath + "/model.config"));
  EXPECT_TRUE(common::isFile(
      modelPath + "/materials/scripts/map_tiles.material"));
  std::string centerTileName = "tile_37.386491_-122.065255.png";
  EXPECT_TRUE(common::isFile(
      modelPath + "/materials/textures/" + centerTileName));

  // done testing, remove cache
  boost::filesystem::remove_all(modelPath);
  EXPECT_FALSE(common::exists(modelPath));

  // restore original model cache if needed
  if (common::exists(modelBackupPath))
  {
    boost::filesystem::rename(modelBackupPath, modelPath);
    EXPECT_FALSE(common::exists(modelBackupPath));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
