/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <string.h>
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/OBJLoader.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class ObjLoaderTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// This tests opening an OBJ file that has an invalid material reference
TEST_F(ObjLoaderTest, InvalidMaterial)
{
  gazebo::common::OBJLoader objLoader;
  gazebo::common::Mesh *mesh = nullptr;

  std::string meshFilename = std::string(TEST_PATH) +
    "/media/models/invalid_material.obj";

  mesh = objLoader.Load(meshFilename);
  EXPECT_TRUE(mesh != nullptr);
}
