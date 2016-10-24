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

#include <gazebo/gazebo_config.h>

#ifdef HAVE_GDAL
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wfloat-equal"
# include <gdalwarper.h>
# include <gdal_priv.h>
# pragma GCC diagnostic pop
#endif

#include "gazebo/common/Console.hh"
#include "gazebo/common/ImageHeightmap.hh"
#include "gazebo/common/HeightmapData.hh"
#include "gazebo/common/Dem.hh"


using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
HeightmapData *HeightmapDataLoader::LoadImageAsTerrain(
    const std::string &_filename)
{
  ImageHeightmap *img = new ImageHeightmap();

  if (img->Load(_filename) != 0)
  {
    gzerr << "Unable to load an image as a terrain [" << _filename << "]\n";
    return nullptr;
  }

  return static_cast<HeightmapData *>(img);
}


#ifdef HAVE_GDAL
//////////////////////////////////////////////////
HeightmapData *HeightmapDataLoader::LoadDEMAsTerrain(
    const std::string &_filename)
{
  Dem *dem = new Dem();
  if (dem->Load(_filename) != 0)
  {
    gzerr << "Unable to load a DEM file as a terrain [" << _filename << "]\n";
    return nullptr;
  }
  return static_cast<HeightmapData *>(dem);
}

//////////////////////////////////////////////////
HeightmapData *HeightmapDataLoader::LoadTerrainFile(
    const std::string &_filename)
{
  // Register the GDAL drivers
  GDALAllRegister();

  GDALDataset *poDataset = reinterpret_cast<GDALDataset *>
      (GDALOpen(_filename.c_str(), GA_ReadOnly));

  if (!poDataset)
  {
    gzerr << "Unrecognized terrain format in file [" << _filename << "]\n";
    return nullptr;
  }

  std::string fileFormat = poDataset->GetDriver()->GetDescription();
  GDALClose(reinterpret_cast<GDALDataset *>(poDataset));

  // Check if the heightmap file is an image
  if (fileFormat == "JPEG" || fileFormat == "PNG")
  {
    // Load the terrain file as an image
    return LoadImageAsTerrain(_filename);
  }
  else
  {
    // Load the terrain file as a DEM
    return LoadDEMAsTerrain(_filename);
  }
}
#else
HeightmapData *HeightmapDataLoader::LoadTerrainFile(
    const std::string &_filename)
{
  // Load the terrain file as an image
  return LoadImageAsTerrain(_filename);
}
#endif
