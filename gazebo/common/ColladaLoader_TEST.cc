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
#include <gtest/gtest.h>

#include "test_config.h"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "test/util.hh"

using namespace gazebo;

class ColladaLoader : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(ColladaLoader, LoadBox)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.dae");

  EXPECT_STREQ("unknown", mesh->GetName().c_str());
  EXPECT_EQ(math::Vector3(1, 1, 1), mesh->GetMax());
  EXPECT_EQ(math::Vector3(-1, -1, -1), mesh->GetMin());
  EXPECT_EQ(36u, mesh->GetVertexCount());
  EXPECT_EQ(36u, mesh->GetNormalCount());
  EXPECT_EQ(36u, mesh->GetIndexCount());
  EXPECT_EQ(0u, mesh->GetTexCoordCount());
  EXPECT_EQ(1u, mesh->GetSubMeshCount());
  EXPECT_EQ(1u, mesh->GetMaterialCount());

  // Make sure we can read a submesh name
  EXPECT_STREQ("Cube", mesh->GetSubMesh(0)->GetName().c_str());
}

/////////////////////////////////////////////////
TEST_F(ColladaLoader, ShareVertices)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.dae");

  for (unsigned int i = 0; i < mesh->GetSubMeshCount(); ++i)
  {
    const common::SubMesh *subMesh = mesh->GetSubMesh(i);
    ASSERT_GT(subMesh->GetVertexCount(), 0u);
    ASSERT_GT(subMesh->GetNormalCount(), 0u);

    for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
    {
      math::Vector3 v = subMesh->GetVertex(j);
      math::Vector3 n = subMesh->GetNormal(j);

      // Verify there is no other vertex with the same position AND normal
      for (unsigned int k = j+1; k < subMesh->GetVertexCount(); ++k)
      {
        if (v == subMesh->GetVertex(k))
        {
          EXPECT_TRUE(n != subMesh->GetNormal(k));
        }
      }
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
