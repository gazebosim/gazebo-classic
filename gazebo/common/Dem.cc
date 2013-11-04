/*
 * Copyright 2012-2013 Open Source Robotics Foundation
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

#include <algorithm>
#include <boost/filesystem.hpp>
#include <gazebo/gazebo_config.h>

#ifdef HAVE_GDAL
# include <gdal/ogr_spatialref.h>
#endif

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Dem.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/math/Angle.hh"

using namespace gazebo;
using namespace common;

#ifdef HAVE_GDAL

//////////////////////////////////////////////////
Dem::Dem()
{
  this->dataSet = NULL;
  GDALAllRegister();
}

//////////////////////////////////////////////////
Dem::~Dem()
{
  this->demData.clear();

  if (this->dataSet)
    GDALClose(reinterpret_cast<GDALDataset *>(this->dataSet));
}

//////////////////////////////////////////////////
int Dem::Load(const std::string &_filename)
{
  unsigned int width;
  unsigned int height;
  int xSize, ySize;
  double upLeftX, upLeftY, upRightX, upRightY, lowLeftX, lowLeftY;
  math::Angle upLeftLat, upLeftLong, upRightLat, upRightLong;
  math::Angle lowLeftLat, lowLeftLong;

  // Sanity check
  std::string fullName = _filename;
  if (!boost::filesystem::exists(boost::filesystem::path(fullName)))
    fullName = common::find_file(_filename);

  if (!boost::filesystem::exists(boost::filesystem::path(fullName)))
  {
    gzerr << "Unable to open DEM file[" << _filename
          << "], check your GAZEBO_RESOURCE_PATH settings." << std::endl;
    return -1;
  }

  this->dataSet = reinterpret_cast<GDALDataset *>(GDALOpen(
    fullName.c_str(), GA_ReadOnly));

  if (this->dataSet == NULL)
  {
    gzerr << "Unable to open DEM file[" << fullName
          << "]. Format not recognised as a supported dataset." << std::endl;
    return -1;
  }

  int nBands = this->dataSet->GetRasterCount();
  if (nBands != 1)
  {
    gzerr << "Unsupported number of bands in file [" << fullName + "]. Found "
          << nBands << " but only 1 is a valid value." << std::endl;
    return -1;
  }

  // Set the pointer to the band
  this->band = this->dataSet->GetRasterBand(1);

  // Raster width and height
  xSize = this->dataSet->GetRasterXSize();
  ySize = this->dataSet->GetRasterYSize();

  // Corner coordinates
  upLeftX = 0.0;
  upLeftY = 0.0;
  upRightX = xSize;
  upRightY = 0.0;
  lowLeftX = 0.0;
  lowLeftY = ySize;

  // Calculate the georeferenced coordinates of the terrain corners
  this->GetGeoReference(upLeftX, upLeftY, upLeftLat, upLeftLong);
  this->GetGeoReference(upRightX, upRightY, upRightLat, upRightLong);
  this->GetGeoReference(lowLeftX, lowLeftY, lowLeftLat, lowLeftLong);

  // Set the world width and height
  this->worldWidth =
     common::SphericalCoordinates::Distance(upLeftLat, upLeftLong,
                                            upRightLat, upRightLong);
  this->worldHeight =
     common::SphericalCoordinates::Distance(upLeftLat, upLeftLong,
                                            lowLeftLat, lowLeftLong);

  // Set the terrain's side (the terrain will be squared after the padding)
  if (math::isPowerOfTwo(ySize - 1))
    height = ySize;
  else
    height = math::roundUpPowerOfTwo(ySize) + 1;

  if (math::isPowerOfTwo(xSize - 1))
    width = xSize;
  else
    width = math::roundUpPowerOfTwo(xSize) + 1;

  this->side = std::max(width, height);

  // Preload the DEM's data
  this->LoadData();

  // Set the min/max heights
  this->minElevation = *std::min_element(&this->demData[0],
      &this->demData[0] + this->side * this->side);
  this->maxElevation = *std::max_element(&this->demData[0],
      &this->demData[0] + this->side * this->side);

  return 0;
}

//////////////////////////////////////////////////
double Dem::GetElevation(double _x, double _y)
{
  if (_x >= this->GetWidth() || _y >= this->GetHeight())
  {
    gzthrow("Illegal coordinates. You are asking for the elevation in (" <<
          _x << "," << _y << ") but the terrain is [" << this->GetWidth() <<
           " x " << this->GetHeight() << "]\n");
  }

  return this->demData.at(_y * this->GetWidth() + _x);
}

//////////////////////////////////////////////////
float Dem::GetMinElevation() const
{
  return minElevation;
}

//////////////////////////////////////////////////
float Dem::GetMaxElevation() const
{
  return maxElevation;
}

//////////////////////////////////////////////////
void Dem::GetGeoReference(double _x, double _y,
                          math::Angle &_latitude, math::Angle &_longitude)
{
  double geoTransf[6];
  if (this->dataSet->GetGeoTransform(geoTransf) == CE_None)
  {
    OGRSpatialReference sourceCs;
    OGRSpatialReference targetCs;
    OGRCoordinateTransformation *cT;
    double xGeoDeg, yGeoDeg;

    // Transform the terrain's coordinate system to WGS84
    char* importString = strdup(this->dataSet->GetProjectionRef());
    sourceCs.importFromWkt(&importString);
    targetCs.SetWellKnownGeogCS("WGS84");
    cT = OGRCreateCoordinateTransformation(&sourceCs, &targetCs);

    xGeoDeg = geoTransf[0] + _x * geoTransf[1] + _y * geoTransf[2];
    yGeoDeg = geoTransf[3] + _x * geoTransf[4] + _y * geoTransf[5];

    cT->Transform(1, &xGeoDeg, &yGeoDeg);

    _latitude.SetFromDegree(yGeoDeg);
    _longitude.SetFromDegree(xGeoDeg);
  }
  else
    gzthrow("Unable to obtain the georeferenced values for coordinates ("
            << _x << "," << _y << ")\n");
}

//////////////////////////////////////////////////
void Dem::GetGeoReferenceOrigin(math::Angle &_latitude, math::Angle &_longitude)
{
  return this->GetGeoReference(0, 0, _latitude, _longitude);
}

//////////////////////////////////////////////////
unsigned int Dem::GetHeight() const
{
  return this->side;
}

//////////////////////////////////////////////////
unsigned int Dem::GetWidth() const
{
  return this->side;
}

//////////////////////////////////////////////////
double Dem::GetWorldWidth() const
{
  return this->worldWidth;
}

//////////////////////////////////////////////////
double Dem::GetWorldHeight() const
{
  return this->worldHeight;
}

//////////////////////////////////////////////////
void Dem::FillHeightMap(int _subSampling, unsigned int _vertSize,
                        const math::Vector3 &_size, const math::Vector3 &_scale,
                        bool _flipY, std::vector<float> &_heights)
{
  unsigned int x, y;
  float h = 0;
  float h1 = 0;
  float h2 = 0;

  // Resize the vector to match the size of the vertices.
  _heights.resize(_vertSize * _vertSize);

  double yf, xf, dy, dx;
  unsigned int y1, y2, x1, x2;
  double px1, px2, px3, px4;

  // Iterate over all the vertices
  for (y = 0; y < _vertSize; ++y)
  {
    yf = y / static_cast<double>(_subSampling);
    y1 = floor(yf);
    y2 = ceil(yf);
    if (y2 >= this->side)
      y2 = this->side - 1;
    dy = yf - y1;

    for (x = 0; x < _vertSize; ++x)
    {
      xf = x / static_cast<double>(_subSampling);
      x1 = floor(xf);
      x2 = ceil(xf);
      if (x2 >= this->side)
        x2 = this->side - 1;
      dx = xf - x1;

      px1 = this->demData[y1 * this->side + x1];
      px2 = this->demData[y1 * this->side + x2];
      h1 = (px1 - ((px1 - px2) * dx));

      px3 = this->demData[y2 * this->side + x1];
      px4 = this->demData[y2 * this->side + x2];
      h2 = (px3 - ((px3 - px4) * dx));

      h = (h1 - ((h1 - h2) * dy) - std::max(0.0f, this->GetMinElevation())) *
          _scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (_size.z < 0)
        h *= -1;

      // Convert to 0 if a NODATA value is found
      if (_size.z >= 0 && h < 0)
        h = 0;

      // Store the height for future use
      if (!_flipY)
        _heights[y * _vertSize + x] = h;
      else
        _heights[(_vertSize - y - 1) * _vertSize + x] = h;
    }
  }
}

//////////////////////////////////////////////////
void Dem::LoadData()
{
    unsigned int destWidth;
    unsigned int destHeight;
    unsigned int nXSize = this->dataSet->GetRasterXSize();
    unsigned int nYSize = this->dataSet->GetRasterYSize();
    float ratio;

    this->demData.resize(this->GetWidth() * this->GetHeight());

    // Scale the terrain keeping the same ratio between width and height
    if (nXSize > nYSize)
    {
      ratio = nXSize / nYSize;
      destWidth = this->side;
      destHeight = destWidth / ratio;
    }
    else
    {
      ratio = nYSize / nXSize;
      destHeight = this->side;
      destWidth = destHeight / ratio;
    }

    // Read the whole raster data and convert it to a GDT_Float32 array
    this->band->RasterIO(GF_Read, 0, 0, nXSize, nYSize, &this->demData[0],
                         destWidth, destHeight, GDT_Float32, 0, 0);
}

#endif
