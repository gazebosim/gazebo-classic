/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/ImageHeightmap.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
ImageHeightmap::ImageHeightmap()
{
}

//////////////////////////////////////////////////
int ImageHeightmap::Load(const std::string &_filename)
{
  if (this->img.Load(_filename) != 0)
  {
    gzerr << "Unable to load image file as a terrain [" << _filename << "]\n";
    return -1;
  }

  return 0;
}

//////////////////////////////////////////////////
void ImageHeightmap::FillHeightMap(int _subSampling,
    unsigned int _vertSize, const math::Vector3 &_size,
    const math::Vector3 &_scale, bool _flipY,
    std::vector<float> &_heights)
{
  // Resize the vector to match the size of the vertices.
  _heights.resize(_vertSize * _vertSize);

  int imgHeight = this->GetHeight();
  int imgWidth = this->GetWidth();

  GZ_ASSERT(imgWidth == imgHeight, "Heightmap image must be square");

  // Bytes per row
  unsigned int pitch = this->img.GetPitch();

  // Bytes per pixel
  unsigned int bpp = pitch / imgWidth;

  unsigned char *data = NULL;
  unsigned int count;
  this->img.GetData(&data, count);

  // Iterate over all the vertices
  for (unsigned int y = 0; y < _vertSize; ++y)
  {
    // yf ranges between 0 and 4
    double yf = y / static_cast<double>(_subSampling);
    int y1 = floor(yf);
    int y2 = ceil(yf);
    if (y2 >= imgHeight)
      y2 = imgHeight-1;
    double dy = yf - y1;

    for (unsigned int x = 0; x < _vertSize; ++x)
    {
      double xf = x / static_cast<double>(_subSampling);
      int x1 = floor(xf);
      int x2 = ceil(xf);
      if (x2 >= imgWidth)
        x2 = imgWidth-1;
      double dx = xf - x1;

      double px1 = static_cast<int>(data[y1 * pitch + x1 * bpp]) / 255.0;
      double px2 = static_cast<int>(data[y1 * pitch + x2 * bpp]) / 255.0;
      float h1 = (px1 - ((px1 - px2) * dx));

      double px3 = static_cast<int>(data[y2 * pitch + x1 * bpp]) / 255.0;
      double px4 = static_cast<int>(data[y2 * pitch + x2 * bpp]) / 255.0;
      float h2 = (px3 - ((px3 - px4) * dx));

      float h = (h1 - ((h1 - h2) * dy)) * _scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (_size.z < 0)
        h = 1.0 - h;

      // Store the height for future use
      if (!_flipY)
        _heights[y * _vertSize + x] = h;
      else
        _heights[(_vertSize - y - 1) * _vertSize + x] = h;
    }
  }

  delete [] data;
}

//////////////////////////////////////////////////
std::string ImageHeightmap::GetFilename() const
{
  return this->img.GetFilename();
}

//////////////////////////////////////////////////
unsigned int ImageHeightmap::GetHeight() const
{
  return this->img.GetHeight();
}

//////////////////////////////////////////////////
unsigned int ImageHeightmap::GetWidth() const
{
  return this->img.GetWidth();
}

//////////////////////////////////////////////////
float ImageHeightmap::GetMaxElevation() const
{
  return this->img.GetMaxColor().r;
}
