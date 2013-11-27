/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include "gazebo/common/ImageHeightmap.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
ImageHeightmap::ImageHeightmap(const std::string &_filename)
  : Image(_filename)
{
}

//////////////////////////////////////////////////
void ImageHeightmap::FillHeightMap(int _subSampling,
    unsigned int _vertSize, const math::Vector3 &_size,
    const math::Vector3 &_scale, bool _flipY,
    std::vector<float> &_heights)
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
  unsigned int bpp = pitch / imgWidth;

  unsigned char *data = NULL;
  unsigned int count;
  this->GetData(&data, count);

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

      px1 = static_cast<int>(data[y1 * pitch + x1 * bpp]) / 255.0;
      px2 = static_cast<int>(data[y1 * pitch + x2 * bpp]) / 255.0;
      h1 = (px1 - ((px1 - px2) * dx));

      px3 = static_cast<int>(data[y2 * pitch + x1 * bpp]) / 255.0;
      px4 = static_cast<int>(data[y2 * pitch + x2 * bpp]) / 255.0;
      h2 = (px3 - ((px3 - px4) * dx));

      h = (h1 - ((h1 - h2) * dy)) * _scale.z;

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
unsigned int ImageHeightmap::GetHeight() const
{
  return gazebo::common::Image::GetHeight();
}

//////////////////////////////////////////////////
unsigned int ImageHeightmap::GetWidth() const
{
  return gazebo::common::Image::GetWidth();
}

//////////////////////////////////////////////////
float ImageHeightmap::GetMaxElevation() const
{
  return this->GetMaxColor().r;
}
