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


unsigned int samples=10;
std::string foutput = "";

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadPaths)
{
  common::SVGLoader loader(samples);
  std::vector<common::SVGPath> paths;
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/paths.svg";
  loader.Parse(filePath, paths);
  
  loader.DumpPaths(paths);
  if(!foutput.empty())
  {
    std::ofstream out(foutput.c_str() );
    loader.DumpPaths(paths, out);
    out.close();
  }

/*
  EXPECT_STREQ("unknown", mesh->GetName().c_str());
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
  EXPECT_STREQ("Cube", mesh->GetSubMesh(0)->GetName().c_str());
*/

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{

  for(size_t i=0; i < (size_t)argc; ++i)
  {
    std::cout << i << " " << argv[i] << std::endl;
  }

  if(argc >= 2)
  {
    std::string s = argv[1];
    try {
      samples = atoi(s.c_str());
    }
    catch(...)
    {
        std::cout << "Can't set sample to " << s << ". Sample is " << samples << std::endl;
    }
  }

  if(argc >= 3)
  {
    // output to save results
    foutput = argv[2];
  }


  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
