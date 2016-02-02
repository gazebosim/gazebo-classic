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

  std::vector<ignition::math::Vector2d> vertices;
  // outside square
  vertices.push_back(ignition::math::Vector2d(0, 0));
  vertices.push_back(ignition::math::Vector2d(1, 0));
  vertices.push_back(ignition::math::Vector2d(1, 1));
  vertices.push_back(ignition::math::Vector2d(0, 1));
  // inside square
  vertices.push_back(ignition::math::Vector2d(0.25, 0.25));
  vertices.push_back(ignition::math::Vector2d(0.25, 0.75));
  vertices.push_back(ignition::math::Vector2d(0.75, 0.75));
  vertices.push_back(ignition::math::Vector2d(0.75, 0.25));

  std::vector<ignition::math::Vector2i> edges;
  edges.push_back(ignition::math::Vector2i(0, 1));
  edges.push_back(ignition::math::Vector2i(1, 2));
  edges.push_back(ignition::math::Vector2i(2, 3));
  edges.push_back(ignition::math::Vector2i(3, 0));

  edges.push_back(ignition::math::Vector2i(4, 5));
  edges.push_back(ignition::math::Vector2i(5, 6));
  edges.push_back(ignition::math::Vector2i(6, 7));
  edges.push_back(ignition::math::Vector2i(7, 4));

  common::Mesh *mesh = new common::Mesh();
  mesh->SetName("extruded");
  common::SubMesh *subMesh = new common::SubMesh();
  mesh->AddSubMesh(subMesh);

  bool result = common::GTSMeshUtils::DelaunayTriangulation(vertices,
                                                            edges,
                                                            subMesh);
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
