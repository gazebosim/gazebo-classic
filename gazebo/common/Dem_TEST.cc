/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include "gazebo/common/Dem.hh"
#include "test_config.h"

using namespace gazebo;

#ifdef HAVE_GDAL

/////////////////////////////////////////////////
TEST(DemTest, MisingFile)
{
  common::Dem dem;
  EXPECT_EQ(-1, dem.Load("/file/shouldn/never/exist.png"));
}

/////////////////////////////////////////////////
TEST(DemTest, NotDem)
{
  common::Dem dem;
  boost::filesystem::path path;

  path = "file://media/materials/scripts/CMakeLists.txt";
  EXPECT_EQ(-1, dem.Load(path.string()));
}

/////////////////////////////////////////////////
TEST(DemTest, UnsupportedDem)
{
  common::Dem dem;
  boost::filesystem::path path;

  path = "file://media/materials/textures/wood.jpg";
  EXPECT_EQ(-1, dem.Load(path.string()));
}

/////////////////////////////////////////////////
TEST(DemTest, NonSquaredDem)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_non_squared.tif";
  EXPECT_EQ(0, dem.Load(path.string()));
}

/////////////////////////////////////////////////
TEST(DemTest, SquaredDem)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(0, dem.Load(path.string()));
}

/////////////////////////////////////////////////
TEST(DemTest, BasicAPI)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(0, dem.Load(path.string()));

  // Check the heights and widths
  EXPECT_EQ(129, static_cast<int>(dem.GetHeight()));
  EXPECT_EQ(129, static_cast<int>(dem.GetWidth()));
  EXPECT_FLOAT_EQ(3981.8, dem.GetWorldHeight());
  EXPECT_FLOAT_EQ(3137.63, dem.GetWorldWidth());
  EXPECT_FLOAT_EQ(65.3583, dem.GetMinElevation());
  EXPECT_FLOAT_EQ(318.441, dem.GetMaxElevation());

  // Check GetElevation()
  unsigned int width = dem.GetWidth();
  unsigned int height = dem.GetHeight();
  EXPECT_FLOAT_EQ(215.82324, dem.GetElevation(0, 0));
  EXPECT_FLOAT_EQ(216.04961, dem.GetElevation(width - 1, 0));
  EXPECT_FLOAT_EQ(142.2274, dem.GetElevation(0, height - 1));
  EXPECT_FLOAT_EQ(209.14784, dem.GetElevation(width - 1, height - 1));

  // Check GetGeoReferenceOrigin()
  double latitude;
  double longitude;
  dem.GetGeoReferenceOrigin(longitude, latitude);
  EXPECT_FLOAT_EQ(38.001667, latitude);
  EXPECT_FLOAT_EQ(-122.22278, longitude);
}

/////////////////////////////////////////////////
TEST(DemTest, FillHeightmap)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(0, dem.Load(path.string()));

  // Use FillHeightMap() to retrieve a vector<float> after some transformations
  int subsampling;
  unsigned vertSize;
  math::Vector3 size;
  math::Vector3 scale;
  bool flipY;
  std::vector<float> elevations;

  subsampling = 2;
  vertSize = (dem.GetWidth() * subsampling) - 1;
  size.x = dem.GetWorldWidth();
  size.y = dem.GetWorldHeight();
  size.z = dem.GetMaxElevation() - dem.GetMinElevation();
  scale.x = size.x / vertSize;
  scale.y = size.y / vertSize;
  if (math::equal(dem.GetMaxElevation(), 0.0f))
    scale.z = fabs(size.z);
  else
    scale.z = fabs(size.z) / dem.GetMaxElevation();
  flipY = false;

  dem.FillHeightMap(subsampling, vertSize, size, scale, flipY, elevations);

  // Check the size of the returned vector
  EXPECT_EQ(vertSize * vertSize, elevations.size());

  // Check the elevation of some control points
  EXPECT_FLOAT_EQ(119.58285, elevations.at(0));
  EXPECT_FLOAT_EQ(114.27753, elevations.at(elevations.size() - 1));
  EXPECT_FLOAT_EQ(148.07137, elevations.at(elevations.size() / 2));
}
#endif

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
