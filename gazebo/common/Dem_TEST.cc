/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/math/Angle.hh"
#include "test_config.h"
#include "test/util.hh"

using namespace gazebo;

class DemTest : public gazebo::testing::AutoLogFixture { };

#ifdef HAVE_GDAL

/////////////////////////////////////////////////
TEST_F(DemTest, MisingFile)
{
  common::Dem dem;
  EXPECT_NE(dem.Load("/file/shouldn/never/exist.png"), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, NotDem)
{
  common::Dem dem;
  boost::filesystem::path path;

  path = "file://media/materials/scripts/CMakeLists.txt";
  EXPECT_NE(dem.Load(path.string()), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, UnsupportedDem)
{
  common::Dem dem;
  boost::filesystem::path path;

  path = "file://media/materials/textures/wood.jpg";
  EXPECT_NE(dem.Load(path.string()), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, NonSquaredDemPortrait)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_portrait.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, NonSquaredDemLandscape)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_landscape.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, SquaredDem)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);
}

/////////////////////////////////////////////////
TEST_F(DemTest, BasicAPI)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);

  // Check the heights and widths
  EXPECT_EQ(129, static_cast<int>(dem.GetHeight()));
  EXPECT_EQ(129, static_cast<int>(dem.GetWidth()));
  EXPECT_FLOAT_EQ(3984.4849, dem.GetWorldHeight());
  EXPECT_FLOAT_EQ(3139.7456, dem.GetWorldWidth());
  EXPECT_FLOAT_EQ(65.3583, dem.GetMinElevation());
  EXPECT_FLOAT_EQ(318.441, dem.GetMaxElevation());

  // Check GetElevation()
  unsigned int width = dem.GetWidth();
  unsigned int height = dem.GetHeight();
  EXPECT_FLOAT_EQ(215.82324, dem.GetElevation(0, 0));
  EXPECT_FLOAT_EQ(216.04961, dem.GetElevation(width - 1, 0));
  EXPECT_FLOAT_EQ(142.2274, dem.GetElevation(0, height - 1));
  EXPECT_FLOAT_EQ(209.14784, dem.GetElevation(width - 1, height - 1));

  // Illegal coordinates
  ASSERT_ANY_THROW(dem.GetElevation(0, height));
  ASSERT_ANY_THROW(dem.GetElevation(width, 0));
  ASSERT_ANY_THROW(dem.GetElevation(width, height));

  // Check GetGeoReferenceOrigin()
  math::Angle latitude, longitude;
  dem.GetGeoReferenceOrigin(latitude, longitude);
  EXPECT_FLOAT_EQ(38.001667, latitude.Degree());
  EXPECT_FLOAT_EQ(-122.22278, longitude.Degree());
}

/////////////////////////////////////////////////
TEST_F(DemTest, FillHeightmap)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;

  path /= "data/dem_squared.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);

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
