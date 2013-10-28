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

#include <gtest/gtest.h>

#include "gazebo/common/DEM.hh"

using namespace gazebo;

#ifdef HAVE_GDAL
TEST(DEMTest, DEM)
{
  std::string basedir = "file://media/materials/";
  common::DEM dem;

  // Unexisting file
  EXPECT_EQ(-1, dem.Load("/file/shouldn/never/exist.png"));

  // Not a DEM file
  EXPECT_EQ(-1, dem.Load(basedir + "scripts/CMakeLists.txt"));

  // DEM file but unsupported format (more than 1 band)
  EXPECT_EQ(-1, dem.Load(basedir + "textures/wood.jpg"));

  // DEM non squared
  EXPECT_EQ(0, dem.Load(basedir + "textures/dem_sample_non_squared.tif"));

  // Load a squared GeoTiff DEM file and exercise the basic API
  EXPECT_EQ(0, dem.Load(basedir + "textures/dem_sample.tif"));
  EXPECT_EQ(129, static_cast<int>(dem.GetHeight()));
  EXPECT_EQ(129, static_cast<int>(dem.GetWidth()));
  EXPECT_FLOAT_EQ(3981.8, dem.GetWorldHeight());
  EXPECT_FLOAT_EQ(3137.63, dem.GetWorldWidth());
  EXPECT_FLOAT_EQ(65.3583, dem.GetMinElevation());
  EXPECT_FLOAT_EQ(318.441, dem.GetMaxElevation());

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
