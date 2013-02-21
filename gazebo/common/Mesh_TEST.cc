/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/math/Vector3.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Mesh.hh"

using namespace gazebo;

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


TEST(MeshTest, Mesh)
{
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.mesh"));
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.3ds"));
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.xml"));

  const common::Mesh *mesh =
    common::MeshManager::Instance()->GetMesh("unit_box");
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(36), mesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetTexCoordCount());
  EXPECT_EQ(static_cast<unsigned int>(0), mesh->GetMaterialCount());

  math::Vector3 center, min, max;
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0, 0, 0));
  EXPECT_TRUE(min == math::Vector3(-.5, -.5, -.5));
  EXPECT_TRUE(max == math::Vector3(.5, .5, .5));


  float *vertArray = NULL;
  int *indArray = NULL;
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

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;

  EXPECT_THROW(mesh->GetSubMesh(1), common::Exception);

  for (i = 0; i < 24; ++i)
  {
    verts.push_back(mesh->GetSubMesh(0)->GetVertex(i));
    norms.push_back(mesh->GetSubMesh(0)->GetNormal(i));
  }

  subMesh->CopyVertices(verts);
  subMesh->CopyNormals(norms);
  EXPECT_TRUE(subMesh->HasVertex(math::Vector3(-.5, -.5, -.5)));
  EXPECT_FALSE(subMesh->HasVertex(math::Vector3(0, 0, 0)));

  newMesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0, 0, 0));
  EXPECT_TRUE(min == math::Vector3(-.5, -.5, -.5));
  EXPECT_TRUE(max == math::Vector3(.5, .5, .5));

  subMesh->SetVertexCount(1);
  subMesh->SetIndexCount(1);
  subMesh->SetNormalCount(1);
  subMesh->SetTexCoordCount(1);

  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetTexCoordCount());

  subMesh->SetVertex(0, math::Vector3(1, 2, 3));
  EXPECT_TRUE(subMesh->GetVertex(0) == math::Vector3(1, 2, 3));

  subMesh->SetTexCoord(0, math::Vector2d(.1, .2));
  EXPECT_TRUE(subMesh->GetTexCoord(0) == math::Vector2d(.1, .2));

  newMesh->GenSphericalTexCoord(math::Vector3(0, 0, 0));
  delete newMesh;

  std::ofstream stlFile("/tmp/gazebo_stl_test.stl", std::ios::out);
  stlFile << asciiSTLBox;
  stlFile.close();

  mesh = common::MeshManager::Instance()->Load("/tmp/gazebo_stl_test-bad.stl");
  EXPECT_EQ(NULL, mesh);

  mesh = common::MeshManager::Instance()->Load("/tmp/gazebo_stl_test.stl");
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0.5, 0.5, 0.5));
  EXPECT_TRUE(min == math::Vector3(0, 0, 0));
  EXPECT_TRUE(max == math::Vector3(1, 1, 1));
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
