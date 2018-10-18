/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/OBJLoader.hh"
#include "test/util.hh"

using namespace gazebo;

class OBJLoaderTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(OBJLoaderTest, LoadObjBox)
{
  common::OBJLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.obj");

  EXPECT_STREQ("unknown", mesh->GetName().c_str());
  EXPECT_EQ(ignition::math::Vector3d(1, 1, 1), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(-1, -1, -1), mesh->Min());
  // 36 vertices
  EXPECT_EQ(36u, mesh->GetVertexCount());
  EXPECT_EQ(36u, mesh->GetNormalCount());
  EXPECT_EQ(36u, mesh->GetIndexCount());
  EXPECT_EQ(0u, mesh->GetTexCoordCount());
  EXPECT_EQ(1u, mesh->GetSubMeshCount());
  EXPECT_EQ(1u, mesh->GetMaterialCount());

  // Make sure we can read the submesh name
  EXPECT_STREQ("Cube_Cube.001", mesh->GetSubMesh(0)->GetName().c_str());

  EXPECT_EQ(mesh->GetMaterialCount(), 1u);

  const common::Material *mat = mesh->GetMaterial(0u);
  ASSERT_TRUE(mat);

  // Make sure we read the material color values
  EXPECT_EQ(mat->GetAmbient(), common::Color(0.0, 0.0, 0.0, 1.0));
  EXPECT_EQ(mat->GetDiffuse(), common::Color(0.512, 0.512, 0.512, 1.0));
  EXPECT_EQ(mat->GetSpecular(), common::Color(0.25, 0.25, 0.25, 1.0));
  EXPECT_DOUBLE_EQ(mat->GetTransparency(), 0.0);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
