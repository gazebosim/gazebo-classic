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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/SDTS.hh"
#include "gazebo/math/Angle.hh"

#define RAD2METER ((180. / M_PI) * 60. * 1852.)

using namespace gazebo;
using namespace common;

#ifdef HAVE_GDAL

//////////////////////////////////////////////////
SDTS::SDTS()
{
  GDALAllRegister();
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
void SDTS::Load(const std::string &_filename)
{
  this->poDataset = reinterpret_cast<GDALDataset *>(GDALOpen(
    _filename.c_str(), GA_ReadOnly));

  if (this->poDataset == NULL)
    gzthrow("Unable to find SDTS file [" + _filename + "]\n");

  if (this->poDataset->GetRasterCount() != 1 &&
      this->poDataset->GetRasterCount() != 3)
    gzthrow("Unsupported band number in file [" << _filename + "]. Found " <<
       this-> poDataset->GetRasterCount() <<
       " but only 1 or 3 are valid values\n");

  double upLeftX = 0.0;
  double upLeftY = 0.0;
  double upRightX = this->poDataset->GetRasterXSize();
  double upRightY = 0.0;
  double lowLeftX = 0.0;
  double lowLeftY = this->poDataset->GetRasterYSize();
  double upLeftXgeo, upLeftYgeo;
  double upRightXgeo, upRightYgeo;
  double lowLeftXgeo, lowLeftYgeo;

  // Calculate the georeferenced coordinates
  this->GetGeoReference(upLeftX, upLeftY, upLeftXgeo, upLeftYgeo);
  this->GetGeoReference(upRightX, upRightY, upRightXgeo, upRightYgeo);
  this->GetGeoReference(lowLeftX, lowLeftY, lowLeftXgeo, lowLeftYgeo);

  // Set the world width and height
  this->worldWidth = this->OgrDistance(upLeftYgeo, upLeftXgeo,
                                       upRightYgeo, upRightXgeo);

  this->worldHeight = this->OgrDistance(upLeftYgeo, upLeftXgeo,
                                        lowLeftYgeo, lowLeftXgeo);

  this->dataType = this->poDataset->GetRasterBand(1)->GetRasterDataType();
  this->numBytesPerPoint = GDALGetDataTypeSize(this->dataType) / 8;
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
  unsigned int nXSize = this->poDataset->GetRasterXSize();
  unsigned int nYSize = this->poDataset->GetRasterYSize();
  unsigned int nBands = poDataset->GetRasterCount();
  std::vector<float*> data_v;

  if (*_data)
    delete [] *_data;

  // The first band starts with index 1
  for (unsigned int i = 1; i <= nBands; ++i)
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
  *_data = new float[_count];

  // Testing
  std::cout << "GetData(). Value at (0,0): " << data_v[0][0] << std::endl;

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
          gzerr << "Found " << nBands << " bands. Only 1 or 3 are supported\n";
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
          gzerr << "Found " << nBands << " bands. Only 1 or 3 are supported\n";
        }
      }
    }
  }

  // Release the temporal array containing the data for each band
  for (unsigned int i = 0; i < data_v.size(); ++i)
  {
    if (data_v[i])
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

float SDTS::GetMin()
{
  GDALRasterBand *poBand = NULL;
  std::vector<double> max_v;

  // Read dimension sizes
  int xsize = this->poDataset->GetRasterXSize();
  int ysize = this->poDataset->GetRasterYSize();
  float *buffer = new float[xsize * ysize];

  // Get a pointer to the current band
  poBand = poDataset->GetRasterBand(1);

  // Read the whole data from the current band
  poBand->RasterIO(GF_Read, 0, 0, xsize, ysize, buffer, xsize, ysize,
      this->dataType, 0, 0);

  // Store the maximum value of this band
  float min = *std::min_element(buffer, buffer + xsize * ysize);
  return min;
}

float SDTS::GetMax()
{
  GDALRasterBand *poBand = NULL;
  std::vector<double> max_v;

  // Read dimension sizes
  int xsize = this->poDataset->GetRasterXSize();
  int ysize = this->poDataset->GetRasterYSize();
  float *buffer = new float[xsize * ysize];

  // Get a pointer to the current band
  poBand = poDataset->GetRasterBand(1);

  // Read the whole data from the current band
  poBand->RasterIO(GF_Read, 0, 0, xsize, ysize, buffer, xsize, ysize,
      this->dataType, 0, 0);

  // Store the maximum value of this band
  float max = *std::max_element(buffer, buffer + xsize * ysize);
  return max;
}

//////////////////////////////////////////////////
float SDTS::GetMaxValue()
{
  /*float maxValue;
  GDALRasterBand *poBand = NULL;
  std::vector<double> max_v;*/

  // Read dimension sizes
  /*int xsize = this->poDataset->GetRasterXSize();
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
    float min = *std::min_element(buffer, buffer + xsize * ysize);
    std::cout << "Min: " << min << " max: " << max << std::endl;
    max_v.push_back(max);
  }

  // Create a normalized color with the max values of each band
  if (max_v.size() == 1)
  {
    double v = max_v[0];
    maxClr.Set(v, v, v);
    std::cout << "Max: " << v << std::endl;
  }
  else if (max_v.size() == 3)
  {
    maxClr.Set(max_v[0] / 255.0, max_v[1] / 255.0, max_v[2] / 255.0);
  }
  else
    gzerr << "Found " << nBands << " bands and only 1 or 3 are supported\n";

  // Release the temporal buffer
  if (buffer)
    delete[] buffer;

  return maxClr;*/
  return this->GetMax();
}

//////////////////////////////////////////////////
int SDTS::GetPitch() const
{
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
void SDTS::FillHeightMap(int _subSampling, unsigned int _vertSize,
    const math::Vector3 &_size, const math::Vector3 &_scale, bool _flipY,
    std::vector<float> &_heights)
{
  unsigned int x, y;
  float h = 0;
  float h1 = 0;
  float h2 = 0;

  std::cout << "Scale: " << _scale << std::endl;

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

  double yf, xf, dy, dx;
  int y1, y2, x1, x2;
  double px1, px2, px3, px4;

  float maxHrel = this->GetMax() - this->GetMin();

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

      px1 = data[y1 * pitch + x1 * bpp];
      px2 = data[y1 * pitch + x2 * bpp];
      h1 = (px1 - ((px1 - px2) * dx));

      px3 = data[y2 * pitch + x1 * bpp];
      px4 = data[y2 * pitch + x2 * bpp];
      h2 = (px3 - ((px3 - px4) * dx));

      //h = (h1 - ((h1 - h2) * dy)) * _scale.z;
      //h = (h1 - ((h1 - h2) * dy)) / 255.0;
      h = (h1 - ((h1 - h2) * dy) - maxHrel) * _scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (_size.z < 0)
        h *= -1;

      // Store the height for future use
      if (!_flipY)
        _heights[y * _vertSize + x] = h;
      else
        _heights[(_vertSize - y - 1) * _vertSize + x] = h;
    }
  }

  if (data)
    delete [] data;
}
#endif
