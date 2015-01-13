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

#include <gtest/gtest.h>

#include "test_config.h"
#include "gazebo/common/SVGLoader.hh"
#include "test/util.hh"

using namespace gazebo;

class SVGLoader : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadPaths)
{
  common::SVGLoader loader;
  std::vector<common::SVGPath> paths;
  loader.Parse(std::string(PROJECT_SOURCE_PATH) + "/test/data/paths.svg", paths);

  loader.Dump_paths(paths);     

/*  EXPECT_STREQ("unknown", mesh->GetName().c_str());
  EXPECT_EQ(math::Vector3(1, 1, 1), mesh->GetMax());
  EXPECT_EQ(math::Vector3(-1, -1, -1), mesh->GetMin());
  // 36 vertices, 24 unique, 12 shared.
  EXPECT_EQ(24u, mesh->GetVertexCount());
  EXPECT_EQ(24u, mesh->GetNormalCount());
  EXPECT_EQ(36u, mesh->GetIndexCount());
  EXPECT_EQ(0u, mesh->GetTexCoordCount());
  EXPECT_EQ(1u, mesh->GetSubMeshCount());
  EXPECT_EQ(1u, mesh->GetMaterialCount());

  // Make sure we can read a submesh name
  EXPECT_STREQ("Cube", mesh->GetSubMesh(0)->GetName().c_str());*/
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
