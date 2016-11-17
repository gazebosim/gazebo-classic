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
#include <boost/filesystem.hpp>

#include "test_config.h"
#include "gazebo/common/ColladaLoader.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/SystemPaths.hh"
#include "test/util.hh"

using namespace gazebo;

class MeshTest : public gazebo::testing::AutoLogFixture { };

std::string asciiSTLBox =
"solid MYSOLID\n\
  facet normal  0.0   0.0  -1.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
      vertex    1.0   0.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0  -1.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   1.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal -1.0   0.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   1.0   1.0\n\
      vertex    0.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal -1.0   0.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   0.0   1.0\n\
      vertex    0.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   1.0   0.0\n\
    outer loop\n\
      vertex    0.0   1.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    1.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   1.0   0.0\n\
    outer loop\n\
      vertex    0.0   1.0   0.0\n\
      vertex    0.0   1.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  1.0   0.0   0.0\n\
    outer loop\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  1.0   0.0   0.0\n\
    outer loop\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    1.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0  -1.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0  -1.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   0.0   1.0\n\
      vertex    0.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0   1.0\n\
    outer loop\n\
      vertex    0.0   0.0   1.0\n\
      vertex    1.0   0.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0   1.0\n\
    outer loop\n\
      vertex    0.0   0.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    0.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
endsolid MYSOLID";


/////////////////////////////////////////////////
TEST_F(MeshTest, Mesh)
{
  // Cleanup test directory.
  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::filesystem::remove_all(paths->DefaultTestPath());
  boost::filesystem::create_directories(paths->DefaultTestPath());

  EXPECT_EQ(nullptr, common::MeshManager::Instance()->Load("break.mesh"));
  EXPECT_EQ(nullptr, common::MeshManager::Instance()->Load("break.3ds"));
  EXPECT_EQ(nullptr, common::MeshManager::Instance()->Load("break.xml"));

  const common::Mesh *mesh =
    common::MeshManager::Instance()->GetMesh("unit_box");
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(36), mesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetTexCoordCount());
  EXPECT_EQ(static_cast<unsigned int>(0), mesh->GetMaterialCount());

  ignition::math::Vector3d center, min, max;
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == ignition::math::Vector3d(0, 0, 0));
  EXPECT_TRUE(min == ignition::math::Vector3d(-.5, -.5, -.5));
  EXPECT_TRUE(max == ignition::math::Vector3d(.5, .5, .5));

  float *vertArray = nullptr;
  int *indArray = nullptr;
  mesh->FillArrays(&vertArray, &indArray);

  int i = 0;
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);

  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(.5, vertArray[i++]);
  EXPECT_FLOAT_EQ(-.5, vertArray[i++]);

  common::Mesh *newMesh = new common::Mesh();
  newMesh->SetName("testBox");
  common::SubMesh *subMesh = new common::SubMesh();
  newMesh->AddSubMesh(subMesh);

  std::vector<ignition::math::Vector3d> verts;
  std::vector<ignition::math::Vector3d> norms;

  EXPECT_THROW(mesh->GetSubMesh(1), common::Exception);

  for (i = 0; i < 24; ++i)
  {
    verts.push_back(mesh->GetSubMesh(0)->Vertex(i));
    norms.push_back(mesh->GetSubMesh(0)->Normal(i));
  }

  subMesh->CopyVertices(verts);
  subMesh->CopyNormals(norms);
  EXPECT_TRUE(subMesh->HasVertex(ignition::math::Vector3d(-.5, -.5, -.5)));
  EXPECT_FALSE(subMesh->HasVertex(ignition::math::Vector3d(0, 0, 0)));

  newMesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == ignition::math::Vector3d(0, 0, 0));
  EXPECT_TRUE(min == ignition::math::Vector3d(-.5, -.5, -.5));
  EXPECT_TRUE(max == ignition::math::Vector3d(.5, .5, .5));

  subMesh->SetVertexCount(1);
  subMesh->SetIndexCount(1);
  subMesh->SetNormalCount(1);
  subMesh->SetTexCoordCount(1);

  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetTexCoordCount());

  subMesh->SetVertex(0, ignition::math::Vector3d(1, 2, 3));
  EXPECT_TRUE(subMesh->Vertex(0) == ignition::math::Vector3d(1, 2, 3));

  subMesh->SetTexCoord(0, ignition::math::Vector2d(.1, .2));
  EXPECT_TRUE(subMesh->TexCoord(0) == ignition::math::Vector2d(.1, .2));

  newMesh->GenSphericalTexCoord(ignition::math::Vector3d(0, 0, 0));
  delete newMesh;

  std::ofstream stlFile((paths->DefaultTestPath() +
      "/gazebo_stl_test.stl").c_str(), std::ios::out);
  stlFile << asciiSTLBox;
  stlFile.close();

  mesh = common::MeshManager::Instance()->Load(
      paths->DefaultTestPath() + "/gazebo_stl_test-bad.stl");
  EXPECT_EQ(nullptr, mesh);

  mesh = common::MeshManager::Instance()->Load(
      paths->DefaultTestPath() + "/gazebo_stl_test.stl");
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == ignition::math::Vector3d(0.5, 0.5, 0.5));
  EXPECT_TRUE(min == ignition::math::Vector3d(0, 0, 0));
  EXPECT_TRUE(max == ignition::math::Vector3d(1, 1, 1));

  // Cleanup test directory.
  boost::filesystem::remove_all(paths->DefaultTestPath());
}

/////////////////////////////////////////////////
// Test centering a submesh.
TEST_F(MeshTest, MeshMove)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box_offset.dae");

  // The default location of the box_offset is not centered
  EXPECT_EQ(ignition::math::Vector3d(5.46554, 2.18039, 4.8431), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(3.46555, 0.180391, 2.8431), mesh->Min());

  mesh->Center(ignition::math::Vector3d::Zero);

  EXPECT_EQ(ignition::math::Vector3d(1.0, 1.0, 1.0), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(-1.0, -1.0, -1.0), mesh->Min());

  mesh->Translate(ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(ignition::math::Vector3d(2.0, 3.0, 4.0), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(0.0, 1.0, 2.0), mesh->Min());
}

/////////////////////////////////////////////////
// Test centering a submesh.
TEST_F(MeshTest, SubMeshCenter)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box_offset.dae");

  // The default location of the box_offest is not centered
  EXPECT_EQ(ignition::math::Vector3d(5.46554, 2.18039, 4.8431), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(3.46555, 0.180391, 2.8431), mesh->Min());

  // Get the Cube submesh
  common::SubMesh submesh(mesh->GetSubMesh("Cube"));

  submesh.Center(ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(ignition::math::Vector3d(0, 1, 2), submesh.Min());
  EXPECT_EQ(ignition::math::Vector3d(2, 3, 4), submesh.Max());

  submesh.Translate(ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(ignition::math::Vector3d(1, 3, 5), submesh.Min());
  EXPECT_EQ(ignition::math::Vector3d(3, 5, 7), submesh.Max());

  // The original mesh should not change
  EXPECT_EQ(ignition::math::Vector3d(5.46554, 2.18039, 4.8431), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(3.46555, 0.180391, 2.8431), mesh->Min());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
