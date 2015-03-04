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


unsigned int samples = 10;
std::string foutput = "";

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadPaths)
{
  common::SVGLoader loader(samples);
  std::vector<common::SVGPath> paths;

  // bad path
  bool success = false;
  std::string bad = "/not/a/file.svg";
  success = loader.Parse(bad, paths);
  EXPECT_EQ(true, success);

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/loader.svg";
  loader.Parse(filePath, paths);

  // useful to see the points on screen
  // loader.DumpPaths(paths, std::cout);

  // or in a file
  if (!foutput.empty())
  {
    std::ofstream out(foutput.c_str() );
    loader.DumpPaths(paths, out);
    out.close();
  }

  // the test file has 3 paths inside
  EXPECT_EQ(3, paths.size());
  common::SVGPath &a = paths[0];
  EXPECT_EQ("letterA", a.id);

  // the letter A has 2 subpaths:
  EXPECT_EQ(2, a.subpaths.size());

  // The hole of A
  // 4 commands
  EXPECT_EQ(4, a.subpaths[0].size());
  // 4 points
  EXPECT_EQ(4, a.polylines[0].size());
  // THe A contour has 9
  EXPECT_EQ(9, a.polylines[1].size());

  // see what's going on
  loader.DumpPaths(paths, std::cout);

  // the second path
  common::SVGPath &p2 = paths[1];
  EXPECT_EQ(1, p2.subpaths.size());
  EXPECT_EQ("path2984", p2.id);

  // 8 commands
  EXPECT_EQ(8, p2.subpaths[0].size());
  // since it has splines, there are more
  // points than commands
  EXPECT_EQ(61, p2.polylines[0].size());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
