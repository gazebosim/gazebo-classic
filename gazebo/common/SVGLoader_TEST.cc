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


// tolerance
// minimum distance between 2 distinct points
double tol = 0.05;

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadPaths)
{
  common::SVGLoader loader(10);
  std::vector<common::SVGPath> paths;

  // bad path
  bool success;
  std::string bad = "/not/a/file.svg";
  success = loader.Parse(bad, paths);
  EXPECT_FALSE(success);

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/loader.svg";
  loader.Parse(filePath, paths);

  // useful to see the points on screen
  // loader.DumpPaths(paths, std::cout);

  // or in a file
  std::string foutput = "";
  if (!foutput.empty())
  {
    std::ofstream out(foutput.c_str() );
    loader.DumpPaths(paths, out);
    out.close();
  }

  // the test file has 3 paths inside
  EXPECT_EQ(3u, paths.size());
  common::SVGPath &a = paths[0];
  EXPECT_EQ("letterA", a.id);

  // the letter A has 2 subpaths:
  EXPECT_EQ(2u, a.subpaths.size());

  // The hole of A
  // 4 commands
  EXPECT_EQ(4u, a.subpaths[0].size());
  // 4 points
  EXPECT_EQ(4u, a.polylines[0].size());
  // The A contour has 9
  EXPECT_EQ(9u, a.polylines[1].size());

  // see what's going on
  loader.DumpPaths(paths, std::cout);

  // the second path
  common::SVGPath &p2 = paths[1];
  EXPECT_EQ(1u, p2.subpaths.size());
  EXPECT_EQ("path2984", p2.id);

  // 8 commands
  EXPECT_EQ(8u, p2.subpaths[0].size());
  // since it has splines, there are more
  // points than commands
  EXPECT_EQ(68u, p2.polylines[0].size());
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, LoadArcs)
{
  // check for arc command support
  // this test loads a file with 2 superimposed 180 degree arc:
  //   * drawn from right to left
  //   * with a round top (looks like an igloo, not a bowl)
  // therefore, y DECREASES (y is down in svg)
  //
  // This test ensures that both path coincide (and not form a circle)

  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/arc_test.svg";
  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  // the test file has 2 paths inside
  EXPECT_EQ(2u, paths.size());
  // each path has the same number of points
  EXPECT_EQ(1u, paths[0].polylines.size());
  EXPECT_EQ(1u, paths[1].polylines.size());
  auto &polyline1 = paths[0].polylines[0];
  auto &polyline2 = paths[1].polylines[0];

  for (auto i = 0u; i != polyline1.size(); ++i)
  {
    // extract points
    auto p1 = polyline1[i];
    auto p2 = polyline2[i];

    EXPECT_NEAR(p1.X(), p2.X(), 0.25);
    EXPECT_NEAR(p1.Y(), p2.Y(), 0.25);
  }

  std::ofstream out("arc_test.html");
  loader.DumpPaths(paths, out);
  out.close();
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, Capsule)
{
  // check for arc command support
  // this test loads a file with 2 superimposed 180 degree arc:
  //   * drawn from right to left
  //   * with a round top (looks like an igloo, not a bowl)
  // therefore, y DECREASES (y is down in svg)
  //
  // This test ensures that both path coincide (and not form a circle)

  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/capsule.svg";

  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  // the test file has 2 paths inside
  std::ofstream out("capsule.html");
  loader.DumpPaths(paths, out);
  out.close();
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, ghost_edges)
{
  // check for invalid edges in svg
  // this test loads a file with a circle made of 2 arcs:
  //
  // The resulting polyline should not have duplicate points

  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;

  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/arc_circle.svg";
  bool success = loader.Parse(filePath, paths);

  // save for inspection
  std::ofstream out("arc_circle.html");
  loader.DumpPaths(paths, out);
  out.close();

  EXPECT_EQ(true, success);
  // the test file has 2 paths inside
  EXPECT_EQ(1u, paths.size());
  EXPECT_EQ(1u, paths[0].polylines.size());
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, ClosedLoops)
{
  // this tests the PathsToClosedPolylines function
  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/chassis.svg";
  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  // save for inspection
  std::ofstream out("chassis.html");
  loader.DumpPaths(paths, out);
  out.close();


  std::vector< std::vector<ignition::math::Vector2d> > closedPolys;
  std::vector< std::vector<ignition::math::Vector2d> > openPolys;

  loader.PathsToClosedPolylines(paths, tol, closedPolys, openPolys);
  EXPECT_EQ(0u, openPolys.size());
  EXPECT_EQ(23u, closedPolys.size());
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, Transforms)
{
  // this tests the scale, matrix and rotation transforms
  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/transform.svg";
  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  // save for inspection
  std::ofstream out("transform.html");
  loader.DumpPaths(paths, out);
  out.close();


  std::vector< std::vector<ignition::math::Vector2d> > closedPolys;
  std::vector< std::vector<ignition::math::Vector2d> > openPolys;

  loader.PathsToClosedPolylines(paths, tol, closedPolys, openPolys);

  EXPECT_EQ(0u, openPolys.size());
  EXPECT_EQ(2u, closedPolys.size());

  // without transform, the first segment of the firts polyline is
  // about 132 units along y
  auto p0 = closedPolys[0][0];
  auto p1 = closedPolys[0][1];
  double dy = fabs(p0.Y() - p1.Y());
  EXPECT_LE(dy, 40.0);
  // without transform, the first segment of the second poly is horizontal
  // (y is constant, dy is 0)
  dy = fabs(closedPolys[1][0].Y() - closedPolys[1][1].Y() );
  EXPECT_GT(dy, 150.0);
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, Transforms2)
{
  // this tests the skewY and skewX transforms
  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/transform2.svg";
  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  // save for inspection
  std::ofstream out("transform2.html");
  loader.DumpPaths(paths, out);
  out.close();


  std::vector< std::vector<ignition::math::Vector2d> > closedPolys;
  std::vector< std::vector<ignition::math::Vector2d> > openPolys;

  loader.PathsToClosedPolylines(paths, tol, closedPolys, openPolys);

  EXPECT_EQ(0u, openPolys.size());
  EXPECT_EQ(3u, closedPolys.size());

  EXPECT_EQ("original", paths[0].id);
  double dxOriginal = fabs(closedPolys[0][0].X() - closedPolys[0][1].X() );
  double dyOriginal = fabs(closedPolys[0][0].Y() - closedPolys[0][1].Y() );
  gzmsg << dxOriginal << " X original 0-1\n";
  gzmsg << dyOriginal << " Y original 0-1\n";
  // the sgment is vertical (dx = 0)
  EXPECT_DOUBLE_EQ(dxOriginal, 0);

  EXPECT_EQ("skewx", paths[1].id);
  double dxSkewX = fabs(closedPolys[1][0].X() - closedPolys[1][1].X() );
  double dySkewX = fabs(closedPolys[1][0].Y() - closedPolys[1][1].Y() );
  gzmsg << dxSkewX << " X skewX 0-1\n";
  gzmsg << dySkewX << " Y skewX 0-1\n";
  // the segment is 45 degree, hence dx = dy
  EXPECT_DOUBLE_EQ(dxSkewX, dySkewX);

  EXPECT_EQ("skewy", paths[2].id);
  double dxSkewY = fabs(closedPolys[2][0].X() - closedPolys[2][1].X() );
  double dySkewY = fabs(closedPolys[2][0].Y() - closedPolys[2][1].Y() );
  gzmsg << dxSkewY << " X skewY 0-1\n";
  gzmsg << dySkewY << " Y skewY 0-1\n";
  // the segment is vertical (dx = 0)
  EXPECT_DOUBLE_EQ(dxOriginal, dxSkewY);

  // segment is skewed 30 degrees
  double dxSkewYb = fabs(closedPolys[2][5].X() - closedPolys[2][4].X() );
  double dySkewYb = fabs(closedPolys[2][5].Y() - closedPolys[2][4].Y() );
  gzmsg << dxSkewYb << " X skewY b\n";
  gzmsg << dySkewYb << " Y skewY b\n";
  EXPECT_GT(dxSkewYb, 131.0);
  EXPECT_GT(dySkewYb, 35.0);
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, Transforms3)
{
  // this tests the skewY and skewX transforms
  common::SVGLoader loader(3);
  std::vector<common::SVGPath> paths;
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/svg/transform3.svg";
  bool success = loader.Parse(filePath, paths);
  EXPECT_EQ(true, success);

  EXPECT_EQ(3u, paths.size());
  EXPECT_EQ("outer_path", paths[0].id);
  EXPECT_EQ("path_no_tx", paths[1].id);
  EXPECT_EQ("path_tx", paths[2].id);

  gzerr << paths[0].id << " tx: " << paths[0].transform << std::endl;
  gzerr << paths[1].id << " tx: " << paths[1].transform << std::endl;
  gzerr << paths[2].id << " tx: " << paths[2].transform << std::endl;

  // save for inspection
  std::ofstream out("transform3.html");
  loader.DumpPaths(paths, out);
  out.close();

  std::vector< std::vector<ignition::math::Vector2d> > closedPolys;
  std::vector< std::vector<ignition::math::Vector2d> > openPolys;
  loader.PathsToClosedPolylines(paths, tol, closedPolys, openPolys);

  EXPECT_EQ(0u, openPolys.size());
  EXPECT_EQ(3u, closedPolys.size());
}

/////////////////////////////////////////////////
TEST_F(SVGLoader, MultipleFiles)
{
  // this test can load multiple svg files and
  // save the html for inspection
  std::vector<std::string> files;
  files.push_back("issue_1489_5.svg");
  for (auto file : files)
  {
    // this tests the skewY and skewX transforms
    common::SVGLoader loader(3);
    std::vector<common::SVGPath> paths;
    std::string filePath = std::string(PROJECT_SOURCE_PATH);
    filePath += "/test/data/svg/";
    filePath += file;
    bool success = loader.Parse(filePath, paths);
    EXPECT_EQ(true, success);

    // save for inspection
    std::ofstream out(file + ".html");
    loader.DumpPaths(paths, out);
    out.close();
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
