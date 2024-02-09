/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/common/Dem.hh"
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
  ignition::math::Angle latitude, longitude;
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
  ignition::math::Vector3d size;
  ignition::math::Vector3d scale;
  bool flipY;
  std::vector<float> elevations;

  subsampling = 2;
  vertSize = (dem.GetWidth() * subsampling) - 1;
  size.X(dem.GetWorldWidth());
  size.Y(dem.GetWorldHeight());
  size.Z(dem.GetMaxElevation() - dem.GetMinElevation());
  scale.X(size.X() / vertSize);
  scale.Y(size.Y() / vertSize);

  if (ignition::math::equal(dem.GetMaxElevation(), 0.0f))
    scale.Z(fabs(size.Z()));
  else
    scale.Z(fabs(size.Z()) / dem.GetMaxElevation());
  flipY = false;

  dem.FillHeightMap(subsampling, vertSize, size, scale, flipY, elevations);

  // Check the size of the returned vector
  EXPECT_EQ(vertSize * vertSize, elevations.size());

  // Check the elevation of some control points
  EXPECT_FLOAT_EQ(184.94113, elevations.at(0));
  EXPECT_FLOAT_EQ(179.63583, elevations.at(elevations.size() - 1));
  EXPECT_FLOAT_EQ(213.42966, elevations.at(elevations.size() / 2));
}

/////////////////////////////////////////////////
TEST_F(DemTest, NegDem)
{
  common::Dem dem;

  boost::filesystem::path path = TEST_PATH;
  path /= "media/materials/textures/dem_neg.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);

  // Check the heights and widths
  EXPECT_EQ(33, static_cast<int>(dem.GetHeight()));
  EXPECT_EQ(33, static_cast<int>(dem.GetWidth()));
  // This DEM model is from the moon. Older versions
  // of libproj will calculate the size assuming it
  // is of the Earth, unless we specify the surface.
  bool sizeSameAsEarth =
    (std::abs(293.51089 - dem.GetWorldWidth()) < 0.1)
    && (std::abs(293.51068 - dem.GetWorldHeight()) < 0.1);
  // Newer versions give invalid sizes, 0 in this case.
  bool invalidSize =
    (dem.GetWorldHeight() < 0.001) &&
    (dem.GetWorldWidth() < 0.001);

  EXPECT_TRUE(sizeSameAsEarth || invalidSize);

  EXPECT_FLOAT_EQ(-212.29616, dem.GetMinElevation());
  EXPECT_FLOAT_EQ(-205.44009, dem.GetMaxElevation());
}

/////////////////////////////////////////////////
TEST_F(DemTest, UnfinishedDem)
{
  common::Dem dem;

  boost::filesystem::path path = TEST_PATH;
  path /= "data/dem_unfinished.tif";
  EXPECT_EQ(dem.Load(path.string()), 0);

  // Check that the min and max elevations are valid for an unfinished
  // and unfilled dem.
  EXPECT_EQ(33, static_cast<int>(dem.GetHeight()));
  EXPECT_EQ(33, static_cast<int>(dem.GetWidth()));
  EXPECT_FLOAT_EQ(111287.59, dem.GetWorldHeight());
  EXPECT_FLOAT_EQ(88878.297, dem.GetWorldWidth());
  // gdal reports min elevation as -32768 but this is treated as a nodata
  // by our dem class and ignored when computing the min elevation
  EXPECT_FLOAT_EQ(-10, dem.GetMinElevation());
  EXPECT_FLOAT_EQ(1909, dem.GetMaxElevation());

  // test another dem file with multiple nodata values
  common::Dem demNoData;

  path = TEST_PATH;
  path /= "data/dem_nodata.dem";
  EXPECT_EQ(demNoData.Load(path.string()), 0);

  // Check that the min and max elevations are valid for a dem with multiple
  // nodata values
  EXPECT_EQ(65, static_cast<int>(demNoData.GetHeight()));
  EXPECT_EQ(65, static_cast<int>(demNoData.GetWidth()));
  EXPECT_FLOAT_EQ(14050.08, demNoData.GetWorldHeight());
  EXPECT_FLOAT_EQ(9785.4375, demNoData.GetWorldWidth());
  // gdal reports min elevation as -32767 but this is treated as a nodata
  // by our dem class and ignored when computing the min elevation
  EXPECT_FLOAT_EQ(682, demNoData.GetMinElevation());
  EXPECT_FLOAT_EQ(2932, demNoData.GetMaxElevation());
}

/////////////////////////////////////////////////
TEST_F(DemTest, LunarDemLoad)
{
  common::Dem dem;
  boost::filesystem::path path = TEST_PATH;
  path /= "data/dem_moon.tif";

  // Sizes will be computed incorrectly
  // as the celestial bodies in DEM file and
  // default spherical coordinates do not match.
  EXPECT_EQ(dem.Load(path.string()), 0);
  
  // Older versions of libproj will default the size
  // calculation to Earth's size.
  bool sizeSameAsEarth =
    (std::abs(293.51 - dem.GetWorldWidth()) < 0.1)
    && (std::abs(293.51 - dem.GetWorldHeight()) < 0.1);
  // Newer versions of libproj will output a zero.
  bool invalidSize =
    (dem.GetWorldWidth() < 0.001) &&
    (dem.GetWorldHeight() < 0.001);

  EXPECT_TRUE(sizeSameAsEarth || invalidSize);

  // Setting the spherical coordinates solves the
  // problem.
  common::SphericalCoordinatesPtr moonSC =
    boost::make_shared<common::SphericalCoordinates>(
        common::SphericalCoordinates::MOON_SCS);

  dem.SetSphericalCoordinates(moonSC);
  EXPECT_EQ(dem.Load(path.string()), 0);

  EXPECT_FLOAT_EQ(80.0417, dem.GetWorldWidth());
  EXPECT_FLOAT_EQ(80.0417, dem.GetWorldHeight());
}
#endif

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
