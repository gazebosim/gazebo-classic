/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Dem.hh"
#include "gazebo/common/HeightmapData.hh"
#include "gazebo/common/ImageHeightmap.hh"

#include "test_config.h"
#include "test/util.hh"

#define ELEVATION_TOL 1e-8

using namespace gazebo;

class HeightmapDataLoaderTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(HeightmapDataLoaderTest, ImageHeightmap)
{
  common::HeightmapData *heightmapData = NULL;
  std::string path;

  path = "file://media/materials/textures/heightmap_bowl.png";

  heightmapData = common::HeightmapDataLoader::LoadTerrainFile(
      common::find_file(path));

  EXPECT_TRUE(heightmapData != nullptr);

  common::ImageHeightmap *img =
      dynamic_cast<common::ImageHeightmap *>(heightmapData);
  EXPECT_TRUE(img != nullptr);

  // Check the heights and widths
  EXPECT_EQ(129, static_cast<int>(img->GetHeight()));
  EXPECT_EQ(129, static_cast<int>(img->GetWidth()));
  EXPECT_NEAR(0.99607843, img->GetMaxElevation(), ELEVATION_TOL);
}

#ifdef HAVE_GDAL
/////////////////////////////////////////////////
TEST_F(HeightmapDataLoaderTest, DemHeightmap)
{
  common::HeightmapData *heightmapData = NULL;

  boost::filesystem::path path = TEST_PATH;
  path /= "data/dem_squared.tif";

  heightmapData = common::HeightmapDataLoader::LoadTerrainFile(path.string());

  EXPECT_TRUE(heightmapData != nullptr);

  common::Dem *dem = dynamic_cast<common::Dem *>(heightmapData);
  EXPECT_TRUE(dem != nullptr);

  // Check the heights and widths
  EXPECT_EQ(129, static_cast<int>(dem->GetHeight()));
  EXPECT_EQ(129, static_cast<int>(dem->GetWidth()));
  EXPECT_FLOAT_EQ(3984.4849, dem->GetWorldHeight());
  EXPECT_FLOAT_EQ(3139.7456, dem->GetWorldWidth());
  EXPECT_FLOAT_EQ(65.3583, dem->GetMinElevation());
  EXPECT_FLOAT_EQ(318.441, dem->GetMaxElevation());

  // Check GetElevation()
  unsigned int width = dem->GetWidth();
  unsigned int height = dem->GetHeight();
  EXPECT_FLOAT_EQ(215.82324, dem->GetElevation(0, 0));
  EXPECT_FLOAT_EQ(216.04961, dem->GetElevation(width - 1, 0));
  EXPECT_FLOAT_EQ(142.2274, dem->GetElevation(0, height - 1));
  EXPECT_FLOAT_EQ(209.14784, dem->GetElevation(width - 1, height - 1));
}
#endif

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
