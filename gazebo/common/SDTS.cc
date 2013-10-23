/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include <cfloat>
#include <gdal/ogr_spatialref.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/SDTS.hh"
#include "gazebo/math/Angle.hh"

#define RAD2METER ((180. / M_PI) * 60. * 1852.)

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
SDTS::SDTS(const std::string &_filename)
{
  GDALAllRegister();

  this->poDataset = reinterpret_cast<GDALDataset *>
      (GDALOpen(_filename.c_str(), GA_ReadOnly));

  if (this->poDataset == NULL)
    gzthrow("Unable to find SDTS file [" + _filename + "]\n");

  if (this->poDataset->GetRasterCount() != 1 &&
      this->poDataset->GetRasterCount() != 3)
    gzthrow("Unsupported band number in file [" << _filename + "]. Found " <<
       this-> poDataset->GetRasterCount() <<
       " but only 1 or 3 are valid values\n");


  // Define Geographic coordinate system - set it to WGS84.
  /*OGRSpatialReference *poSRS_Geog = new OGRSpatialReference();
  poSRS_Geog->importFromEPSG(4326); // WGS84

  // Define Projected coordinate system - set to the GeoTransform.
  const char *sProj = this->poDataset->GetProjectionRef();
  OGRSpatialReference *poSRS_Proj = new OGRSpatialReference(sProj);

  // Set up the coordinate transform (projected-to-geographic).
  OGRCoordinateTransformation *poCT_Proj2Geog;
  poCT_Proj2Geog = OGRCreateCoordinateTransformation(poSRS_Proj, poSRS_Geog);

  // Now everything is set up and we set transforming coordinates!
  // Pass Lon/Lat coordinates to the Transform function:
  double x = 0;
  double y = 0;
  poCT_Proj2Geog->Transform(1, &x, &y);*/

  double upperLeftX = 0.0;
  double upperLeftY = 0.0;
  double upperRightX = this->poDataset->GetRasterXSize();
  double upperRightY = 0.0;
  double lowerLeftX = 0.0;
  double lowerLeftY = this->poDataset->GetRasterYSize();
  double upperLeftXgeo, upperLeftYgeo;
  double upperRightXgeo, upperRightYgeo;
  double lowerLeftXgeo, lowerLeftYgeo;

  // Calculate the georeferenced coordinates
  this->GetGeoReference(upperLeftX, upperLeftY, upperLeftXgeo, upperLeftYgeo);
  this->GetGeoReference(upperRightX, upperRightY, upperRightXgeo, upperRightYgeo);
  this->GetGeoReference(lowerLeftX, lowerLeftY, lowerLeftXgeo, lowerLeftYgeo);
  
  /*std::cout << "Upper left: (" << upperLeftXgeo << "," << upperLeftYgeo << ")\n";
  std::cout << "Upper right: (" << upperRightXgeo << "," << upperRightYgeo << ")\n";
  std::cout << "Lower left: (" << lowerLeftXgeo << "," << lowerLeftYgeo << ")\n";*/

  // Set the world width and height
  this->worldWidth = this->OgrDistance(upperLeftYgeo, upperLeftXgeo,
                                       upperRightYgeo, upperRightXgeo);

  this->worldHeight = this->OgrDistance(upperLeftYgeo, upperLeftXgeo,
                                        lowerLeftYgeo, lowerLeftXgeo);

  std::cout << "World width: " << this->worldWidth << std::endl;
  std::cout << "World height: " << this->worldHeight << std::endl;

  this->dataType = this->poDataset->GetRasterBand(1)->GetRasterDataType();
  std::cout << "Data type: " << this->dataType << std::endl;

  this->numBytesPerPoint = GDALGetDataTypeSize(this->dataType) / 8;
  std::cout << "Num of bytes per cell: " << this->numBytesPerPoint << std::endl;
}

void SDTS::GetGeoReference(double _pixel, double _line,
                           double &_xGeo, double &_yGeo)
{
  double adfGeoTransform[6];
  if (this->poDataset->GetGeoTransform(adfGeoTransform) == CE_None)
  {
    _xGeo = adfGeoTransform[0] +
            _pixel * adfGeoTransform[1] + _line * adfGeoTransform[2];
    _yGeo = adfGeoTransform[3] +
            _pixel * adfGeoTransform[4] + _line * adfGeoTransform[5];
  }  
}

//////////////////////////////////////////////////
SDTS::~SDTS()
{
}

//////////////////////////////////////////////////
unsigned int SDTS::GetBPP() const
{
  return this->poDataset->GetRasterCount();
}

//////////////////////////////////////////////////
void SDTS::GetData(float **_data, unsigned int &_count) const
{
  GDALRasterBand  *poBand;
  int nXSize = this->poDataset->GetRasterXSize();
  int nYSize = this->poDataset->GetRasterYSize();
  int nBands = poDataset->GetRasterCount();
  std::vector<float*> data_v;

  if (*_data)
    delete [] *_data;

  // The first band starts with index 1
  for (int i = 1; i <= nBands; ++i)
  {
    // Get a pointer to the current band
    poBand = poDataset->GetRasterBand(i);

    // Read the whole data from the current band
    float *buffer = new float[nXSize * nYSize];
    poBand->RasterIO(GF_Read, 0, 0, nXSize, nYSize, buffer, nXSize, nYSize,
        this->dataType, 0, 0);

    // Store the pointer to this band's data for the future
    data_v.push_back(buffer);
  }

  // Allocate memory for the array that will contain all the data
  _count = this->GetWidth() * this->GetHeight() * nBands;
  std::cout << _count << "\n";
  *_data = new float[_count];

  // Fill the array aligning the data
  float *p = *_data;
  for (unsigned int i = 0; i < this->GetHeight(); ++i)
  {
    for (unsigned int j = 0; j < this->GetWidth(); ++j)
    {
      // Padding
      if ((i >= nXSize) || (j >= nYSize))
      {
        if (nBands == 1)
        {
          p[0] = 0;
        }
        else if (nBands == 3)
        {
          p[0] = 0;
          p[1] = 0;
          p[2] = 0;
        }
        else
        {
          gzerr << "Found " << nBands << " bands and only 1 or 3 are supported\n";
        }
      }
      else
      {
        int index = i * this->poDataset->GetRasterXSize() + j;
        if (nBands == 1)
        {
          p[0] = data_v[0][index];
          p++;
        }
        else if (nBands == 3)
        {
          p[0] = data_v[0][index];
          p[1] = data_v[1][index];
          p[2] = data_v[2][index];
          p += 3;
        }
        else
        {
          gzerr << "Found " << nBands << " bands and only 1 or 3 are supported\n";
        }
      }
    }
  }

  // Release the temporal array containing the data for each band
  for (unsigned int i = 0; i < data_v.size(); ++i)
  {
    delete[] data_v[i];
  }
}

//////////////////////////////////////////////////
unsigned int SDTS::GetHeight() const
{
  unsigned int currentHeight = this->poDataset->GetRasterYSize();
  if (math::isPowerOfTwo(currentHeight - 1))
    return currentHeight;
  else
    return math::roundUpPowerOfTwo(currentHeight) + 1;
}

//////////////////////////////////////////////////
Color SDTS::GetMaxColor()
{
  Color maxClr;
  GDALRasterBand *poBand = NULL;
  std::vector<double> max_v;

  maxClr.Set(0, 0, 0, 0);

  // Read dimension sizes
  int xsize = this->poDataset->GetRasterXSize();
  int ysize = this->poDataset->GetRasterYSize();
  float *buffer = new float[xsize * ysize];
  int nBands = poDataset->GetRasterCount();

  // The first band starts with index 1
  for (int i = 1; i <= nBands; ++i)
  {
    // Get a pointer to the current band
    poBand = poDataset->GetRasterBand(i);

    // Read the whole data from the current band
    poBand->RasterIO(GF_Read, 0, 0, xsize, ysize, buffer, xsize, ysize,
        this->dataType, 0, 0);

    // Store the maximum value of this band
    float max = *std::max_element(buffer, buffer + xsize * ysize);
    max_v.push_back(max);
  }

  // Create a normalized color with the max values of each band
  if (max_v.size() == 1)
  {
    double v = max_v[0] / poBand->GetMaximum();
    maxClr.Set(v, v, v);
  }
  else if (max_v.size() == 3)
  {
    maxClr.Set(max_v[0] / 255.0, max_v[1] / 255.0, max_v[2] / 255.0);
  }
  else
    gzerr << "Found " << nBands << " bands and only 1 or 3 are supported\n";

  return maxClr;
}

//////////////////////////////////////////////////
int SDTS::GetPitch() const
{
  std::cout << "BPP: " << this->GetBPP() << std::endl;
  std::cout << "Width: " << this->GetWidth() << std::endl;
  std::cout << "Pitch: " << this->GetWidth() * this->GetBPP() << std::endl;
  return this->GetWidth() * this->GetBPP();
}

//////////////////////////////////////////////////
unsigned int SDTS::GetWidth() const
{
  unsigned int currentWidth = this->poDataset->GetRasterXSize();
  if (math::isPowerOfTwo(currentWidth - 1))
    return currentWidth;
  else
    return math::roundUpPowerOfTwo(currentWidth) + 1;
}

//////////////////////////////////////////////////
double SDTS::GetWorldWidth()
{
  return this->worldWidth;
}

//////////////////////////////////////////////////
double SDTS::GetWorldHeight()
{
  return this->worldHeight;
}

//////////////////////////////////////////////////
double SDTS::OgrDistance(double _latAdeg, double _lonAdeg,
                         double _latBdeg, double _lonBdeg)
{
  double latArad, latBrad;
  double cosA, cosB, sinA, sinB, cosP;
  double cosAngle;
 
  cosP = cos(GZ_DTOR(_lonBdeg - _lonAdeg));
  latArad = GZ_DTOR(_latAdeg);
  latBrad = GZ_DTOR(_latBdeg);
  cosA = cos(latArad);
  sinA = sin(latArad);
  cosB = cos(latBrad);
  sinB = sin(latBrad);
  cosAngle = sinA * sinB + cosA * cosB * cosP;

  return this->SafeAcos(cosAngle) * RAD2METER;
}

//////////////////////////////////////////////////
double SDTS::SafeAcos(double _x)
{
  if (_x > 1)
    _x = 1;
  else if (_x < -1)
    _x = -1;
  return acos(_x);
}

//////////////////////////////////////////////////
void SDTS::FillHeightMap(std::vector<float> &_heights,
    int _subSampling, unsigned int _vertSize, const math::Vector3 &_size, 
    const math::Vector3 &_scale, bool _flipY)
{
  unsigned int x, y;
  float h = 0;
  float h1 = 0;
  float h2 = 0;

  // Resize the vector to match the size of the vertices.
  _heights.resize(_vertSize * _vertSize);

  common::Color pixel;

  int imgHeight = this->GetHeight();
  int imgWidth = this->GetWidth();

  GZ_ASSERT(imgWidth == imgHeight, "Heightmap image must be square");

  // Bytes per row
  unsigned int pitch = this->GetPitch();

  // Bytes per pixel
  unsigned int bpp = this->GetBPP();

  float *data = NULL;
  unsigned int count;
  this->GetData(&data, count);
  std::cout << "Count: " << count << std::endl;

  double yf, xf, dy, dx;
  int y1, y2, x1, x2;
  double px1, px2, px3, px4;

  // Iterate over all the vertices
  for (y = 0; y < _vertSize; ++y)
  {
    // yf ranges between 0 and 4
    yf = y / static_cast<double>(_subSampling);
    y1 = floor(yf);
    y2 = ceil(yf);
    if (y2 >= imgHeight)
      y2 = imgHeight-1;
    dy = yf - y1;

    for (x = 0; x < _vertSize; ++x)
    {
      xf = x / static_cast<double>(_subSampling);
      x1 = floor(xf);
      x2 = ceil(xf);
      if (x2 >= imgWidth)
        x2 = imgWidth-1;
      dx = xf - x1;

      px1 = 100 * (data[y1 * pitch + x1 * bpp] / 1000);
      px2 = 100 * (data[y1 * pitch + x2 * bpp] / 1000);
      h1 = (px1 - ((px1 - px2) * dx));

      //std::cout << "Accessing " << x1 << "," << y2 << "\n";
      //std::cout << "Accessing " << x2 << "," << y2 << "\n";
      px3 = 100 * (data[y2 * pitch + x1 * bpp] / 1000);
      px4 = 100 * (data[y2 * pitch + x2 * bpp] / 1000);
      h2 = (px3 - ((px3 - px4) * dx));

      h = (h1 - ((h1 - h2) * dy)) * _scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (_size.z < 0)
        h = 1.0 - h;

      std::cout << "Height: " << h << std::endl;
      // Store the height for future use
      if (!_flipY)
        _heights[y * _vertSize + x] = h;
      else
        _heights[(_vertSize - y - 1) * _vertSize + x] = h;
    }
  }

  delete [] data;
}
