/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/common/MeshManager.hh"
#include "gazebo/gazebo_config.h"
#include "test/util.hh"

using namespace gazebo;

class MeshManager : public gazebo::testing::AutoLogFixture { };

#ifdef HAVE_GTS
/////////////////////////////////////////////////
TEST_F(MeshManager, CreateExtrudedPolyline)
{
  // test extrusion of a path with two subpaths:
  // a smaller square inside a bigger square.
  // The smaller square should be treated as a hole inside the bigger square.
  std::vector<std::vector<ignition::math::Vector2d> > path;
  std::vector<ignition::math::Vector2d> subpath01;
  subpath01.push_back(ignition::math::Vector2d(0, 0));
  subpath01.push_back(ignition::math::Vector2d(1, 0));
  subpath01.push_back(ignition::math::Vector2d(1, 1));
  subpath01.push_back(ignition::math::Vector2d(0, 1));
  subpath01.push_back(ignition::math::Vector2d(0, 0));

  std::vector<ignition::math::Vector2d> subpath02;
  subpath02.push_back(ignition::math::Vector2d(0.25, 0.25));
  subpath02.push_back(ignition::math::Vector2d(0.25, 0.75));
  subpath02.push_back(ignition::math::Vector2d(0.75, 0.75));
  subpath02.push_back(ignition::math::Vector2d(0.75, 0.25));
  subpath02.push_back(ignition::math::Vector2d(0.25, 0.25));

  path.push_back(subpath01);
  path.push_back(subpath02);

  std::string meshName = "extruded_path";
  double height = 10.0;
  common::MeshManager::Instance()->CreateExtrudedPolyline(
      meshName, path, height);

  // check mesh
  EXPECT_TRUE(common::MeshManager::Instance()->HasMesh(meshName));
  const common::Mesh *mesh = common::MeshManager::Instance()->GetMesh(meshName);
  EXPECT_TRUE(mesh != nullptr);

  unsigned int submeshCount = mesh->GetSubMeshCount();
  EXPECT_EQ(submeshCount, 1u);

  // check submesh bounds
  const common::SubMesh *submesh = mesh->GetSubMesh(0);
  EXPECT_TRUE(submesh != nullptr);
  EXPECT_EQ(ignition::math::Vector3d(0, 0, 0), submesh->Min());
  EXPECT_EQ(ignition::math::Vector3d(1.0, 1.0, 10.0), submesh->Max());

  // check vertices
  for (unsigned int i = 0; i < submesh->GetVertexCount(); ++i)
  {
    ignition::math::Vector3d v = submesh->Vertex(i);

    // check no vertices are in the region of the hole
    EXPECT_FALSE((v.X() > 0.25 && v.X() < 0.75));
    EXPECT_FALSE((v.Y() > 0.25 && v.Y() < 0.75));

    // check extruded height
    EXPECT_TRUE((ignition::math::equal(v.Z(), 0.0) ||
          ignition::math::equal(v.Z(), 10.0)));
  }

  // verify same number of normals and vertices
  EXPECT_EQ(submesh->GetVertexCount(), submesh->GetNormalCount());

  // check normals
  for (unsigned int i = 0; i < submesh->GetNormalCount(); ++i)
  {
    ignition::math::Vector3d v = submesh->Vertex(i);
    ignition::math::Vector3d n = submesh->Normal(i);

    // vertex at 0 could be a bottom face or side face
    if (ignition::math::equal(v.Z(), 0.0))
    {
      if (ignition::math::equal(n.Z(), 0.0))
      {
        // side face - check non-zero normal
        EXPECT_TRUE(!(ignition::math::equal(n.X(), 0.0) &&
              ignition::math::equal(n.Y(), 0.0)));
      }
      else
      {
        // bottom face - normal in -z direction
        EXPECT_TRUE((n == -ignition::math::Vector3d::UnitZ) ||
            (ignition::math::equal(n.Z(), 0.0)));
      }
    }

    // vertex at height could be a top face or side face
    if (ignition::math::equal(v.Z(), 10.0))
    {
      if (ignition::math::equal(n.Z(), 0.0))
      {
        // side face - check non-zero normal
        EXPECT_TRUE(!(ignition::math::equal(n.X(), 0.0) &&
              ignition::math::equal(n.Y(), 0.0)));
      }
      else
      {
        // top face - normal in +z direction
        EXPECT_TRUE((n == ignition::math::Vector3d::UnitZ) ||
            (ignition::math::equal(n.Z(), 0.0)));
      }
    }
  }
}

/////////////////////////////////////////////////
TEST_F(MeshManager, CreateExtrudedPolylineClosedPath)
{
  // test extrusion of a path that has two closed subpaths, i.e.,
  // first and last vertices are the same.
  // The following two subpaths form the letter 'A'.
  std::vector<std::vector<ignition::math::Vector2d> > path2;
  std::vector<ignition::math::Vector2d> subpath03;
  subpath03.push_back(ignition::math::Vector2d(2.27467, 1.0967));
  subpath03.push_back(ignition::math::Vector2d(1.81094, 2.35418));
  subpath03.push_back(ignition::math::Vector2d(2.74009, 2.35418));

  std::vector<ignition::math::Vector2d> subpath04;
  subpath04.push_back(ignition::math::Vector2d(2.08173, 0.7599));
  subpath04.push_back(ignition::math::Vector2d(2.4693, 0.7599));
  subpath04.push_back(ignition::math::Vector2d(3.4323, 3.28672));
  subpath04.push_back(ignition::math::Vector2d(3.07689, 3.28672));
  subpath04.push_back(ignition::math::Vector2d(2.84672, 2.63851));
  subpath04.push_back(ignition::math::Vector2d(1.7077, 2.63851));
  subpath04.push_back(ignition::math::Vector2d(1.47753, 3.28672));
  subpath04.push_back(ignition::math::Vector2d(1.11704, 3.28672));

  path2.push_back(subpath03);
  path2.push_back(subpath04);

  std::string meshName = "extruded_path_closed";
  double height = 2.0;
  common::MeshManager::Instance()->CreateExtrudedPolyline(
      meshName, path2, height);

  // check mesh
  EXPECT_TRUE(common::MeshManager::Instance()->HasMesh(meshName));
  const common::Mesh *mesh = common::MeshManager::Instance()->GetMesh(meshName);
  EXPECT_TRUE(mesh != nullptr);

  unsigned int submeshCount = mesh->GetSubMeshCount();
  EXPECT_EQ(submeshCount, 1u);

  // check submesh bounds
  const common::SubMesh *submesh = mesh->GetSubMesh(0);
  EXPECT_TRUE(submesh != nullptr);
  EXPECT_EQ(submesh->Min(), ignition::math::Vector3d(1.11704, 0.7599, 0));
  EXPECT_EQ(submesh->Max(), ignition::math::Vector3d(3.4323, 3.28672, 2.0));

  for (unsigned int i = 0; i < submesh->GetVertexCount(); ++i)
  {
    ignition::math::Vector3d v = submesh->Vertex(i);

    // check no vertices are in the region of the hole using a point-in-polygon
    // algorithm
    bool pointInPolygon = false;
    for (unsigned int j = 0, k = subpath03.size()-1; j < subpath03.size();
        k = ++j)
    {
      if (((subpath03[j].Y() > v.Y()) != (subpath03[k].Y() > v.Y())) &&
          (v.X() < (subpath03[k].X()-subpath03[j].X()) *
           (v.Y()-subpath03[j].Y()) /
         (subpath03[k].Y()-subpath03[j].Y()) + subpath03[j].X()) )
      {
       pointInPolygon = !pointInPolygon;
      }
    }
    EXPECT_FALSE(pointInPolygon);

    // check extruded height
    EXPECT_TRUE((ignition::math::equal(v.Z(), 0.0) ||
          ignition::math::equal(v.Z(), 2.0)));
  }

  // verify same number of normals and vertices
  EXPECT_EQ(submesh->GetVertexCount(), submesh->GetNormalCount());

  // check normals
  for (unsigned int i = 0; i < submesh->GetNormalCount(); ++i)
  {
    ignition::math::Vector3d v = submesh->Vertex(i);
    ignition::math::Vector3d n = submesh->Normal(i);

    // vertex at 0 could be a bottom face or side face
    if (ignition::math::equal(v.Z(), 0.0))
    {
      if (ignition::math::equal(n.Z(), 0.0))
      {
        // side face - check non-zero normal
        EXPECT_TRUE(!(ignition::math::equal(n.X(), 0.0) &&
                      ignition::math::equal(n.Y(), 0.0)));
      }
      else
      {
        // bottom face - normal in -z direction
        EXPECT_TRUE((n == -ignition::math::Vector3d::UnitZ) ||
                    (ignition::math::equal(n.Z(), 0.0)));
      }
    }

    // vertex at height could be a top face or side face
    if (ignition::math::equal(v.Z(), 10.0))
    {
      if (ignition::math::equal(n.Z(), 0.0))
      {
        // side face - check non-zero normal
        EXPECT_TRUE(!(ignition::math::equal(n.X(), 0.0) &&
                      ignition::math::equal(n.Y(), 0.0)));
      }
      else
      {
        // top face - normal in +z direction
        EXPECT_TRUE((n == ignition::math::Vector3d::UnitZ) ||
                    (ignition::math::equal(n.Z(), 0.0)));
      }
    }
  }
}
#endif

/////////////////////////////////////////////////
TEST_F(MeshManager, CreateExtrudedPolylineInvalid)
{
  // test extruding invalid polyline
  std::vector<std::vector<ignition::math::Vector2d> > path;
  std::vector<ignition::math::Vector2d> subpath01;
  subpath01.push_back(ignition::math::Vector2d(0, 0));
  subpath01.push_back(ignition::math::Vector2d(0, 1));
  subpath01.push_back(ignition::math::Vector2d(0, 2));

  path.push_back(subpath01);

  std::string meshName = "extruded_path_invalid";
  double height = 10.0;
  common::MeshManager::Instance()->CreateExtrudedPolyline(
      meshName, path, height);

  // check mesh does not exist due to extrusion failure
  EXPECT_TRUE(!common::MeshManager::Instance()->HasMesh(meshName));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
