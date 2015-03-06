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
#include "gazebo/common/GTSMeshUtils.hh"
#include "test/util.hh"

using namespace gazebo;

class GTSMeshUtils : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(GTSMeshUtils, DelaunayTriangulation)
{
  // test triangulation of a path with two subpaths:
  // a smaller square inside a bigger square.
  // The smaller square should be treated as a hole inside the bigger square.

  std::vector<std::vector<math::Vector2d> > path;
  std::vector<math::Vector2d> subpath01;
  subpath01.push_back(math::Vector2d(0, 0));
  subpath01.push_back(math::Vector2d(1, 0));
  subpath01.push_back(math::Vector2d(1, 1));
  subpath01.push_back(math::Vector2d(0, 1));

  std::vector<math::Vector2d> subpath02;
  subpath02.push_back(math::Vector2d(0.25, 0.25));
  subpath02.push_back(math::Vector2d(0.25, 0.75));
  subpath02.push_back(math::Vector2d(0.75, 0.75));
  subpath02.push_back(math::Vector2d(0.75, 0.25));

  path.push_back(subpath01);
  path.push_back(subpath02);

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName("extruded");
  common::SubMesh *subMesh = new common::SubMesh();
  mesh->AddSubMesh(subMesh);

  bool result = common::GTSMeshUtils::DelaunayTriangulation(path, subMesh);

  EXPECT_TRUE(result);

  // same as number of vertices in the path
  EXPECT_EQ(subMesh->GetVertexCount(), 8u);

  // there should be 8 triangles.
  EXPECT_EQ(subMesh->GetIndexCount() / 3u, 8u);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
