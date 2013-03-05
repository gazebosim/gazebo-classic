/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: Heightmap shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#include <string.h>
#include <math.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/Common.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/HeightmapShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
HeightmapShape::HeightmapShape(CollisionPtr _parent)
    : Shape(_parent)
{
  this->AddType(Base::HEIGHTMAP_SHAPE);
}

//////////////////////////////////////////////////
HeightmapShape::~HeightmapShape()
{
}

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::string filename = common::find_file(this->sdf->GetValueString("uri"));
  if (filename.empty())
  {
    gzthrow("Unable to find heightmap[" +
            this->sdf->GetValueString("uri") + "]\n");
  }

  // Use the image to get the size of the heightmap
  this->img.Load(filename);

  if (this->img.GetWidth() != this->img.GetHeight() ||
      !math::isPowerOfTwo(this->img.GetWidth()-1))
  {
    gzthrow("Heightmap image size must be square, with a size of 2^n+1\n");
  }
}

//////////////////////////////////////////////////
int HeightmapShape::GetSubSampling() const
{
  return this->subSampling;
}

//////////////////////////////////////////////////
void HeightmapShape::Init()
{
  this->subSampling = 1;

  math::Vector3 terrainSize = this->GetSize();

  // sampling size along image width and height
  this->vertSize = this->img.GetWidth() * this->subSampling;
  this->scale.x = terrainSize.x / this->vertSize;
  this->scale.y = terrainSize.y / this->vertSize;

  if (math::equal(this->img.GetMaxColor().r, 0.0f))
    this->scale.z = fabs(terrainSize.z);
  else
    this->scale.z = fabs(terrainSize.z) / this->img.GetMaxColor().r;

  std::cout << "Vert Size[" << this->vertSize << "] Scale[" << this->scale
            << "] Terrain Size[" << terrainSize << "] MaxCOlor[" << this->img.GetMaxColor().r << "]\n";

  // Step 1: Construct the heightmap lookup table
  this->FillHeightMap();
}

//////////////////////////////////////////////////
void HeightmapShape::FillHeightMap()
{
  unsigned int x, y;
  float h = 0;
  // float h1 = 0;
  // float h2 = 0;

  // Resize the vector to match the size of the vertices
  this->heights.resize(this->vertSize * this->vertSize);

  // this->img.Rescale(this->vertSize, this->vertSize);
  common::Color pixel;

  // double yf, xf, dy, dx;
  //int y1, y2, x1, x2;
  double px1;//, px2, px3, px4;

  int imgHeight = this->img.GetHeight();
  int imgWidth = this->img.GetWidth();

  // Bytes per row
  unsigned int pitch = this->img.GetPitch();

  // Bytes per pixel
  unsigned int bpp = pitch / imgWidth;

  std::cout << "Image WxH[" << imgWidth << " " << imgHeight << "] Pitch["
            << pitch << "] BPP[" << bpp << "]\n";

  unsigned char *data = NULL;
  unsigned int count;
  this->img.GetData(&data, count);

  for (y = 0; y < this->vertSize; ++y)
  {
    for (x = 0; x < this->vertSize; ++x)
    {
      px1 = static_cast<int>(data[y * pitch + x * bpp]) / 256.0;
      // std::cout << "XY[" << x << " " << y << "] H[" << px1 << "] \n";

      h = px1 * this->scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (this->GetSize().z < 0)
        h = 1.0 - h;

      // Store the height for future use
      this->heights[y * this->vertSize + x] = h;
    }
  }

  // Iterate over all the vertices
  /*for (y = 0; y < this->vertSize; y++)
  {
    // yf ranges between 0 and 4
    yf = y / static_cast<double>(this->subSampling);
    y1 = floor(yf);
    y2 = ceil(yf);
    if (y2 >= imgHeight)
      y2 = imgHeight-1;
    dy = yf - y1;

    for (x = 0; x < this->vertSize; x++)
    {
      xf = x / static_cast<double>(this->subSampling);
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


      h = (h1 - ((h1 - h2) * dy)) * this->scale.z;

      // invert pixel definition so 1=ground, 0=full height,
      //   if the terrain size has a negative z component
      //   this is mainly for backward compatibility
      if (this->GetSize().z < 0)
        h = 1.0 - h;

      // Store the height for future use
      this->heights[y * this->vertSize + x] = h;
    }
  }*/

  delete [] data;
}

//////////////////////////////////////////////////
std::string HeightmapShape::GetURI() const
{
  return this->sdf->GetValueString("uri");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetSize() const
{
  return this->sdf->GetValueVector3("size");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetPos() const
{
  return this->sdf->GetValueVector3("pos");
}

//////////////////////////////////////////////////
void HeightmapShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::HEIGHTMAP);
  msgs::Set(_msg.mutable_heightmap()->mutable_image(),
            common::Image(this->GetURI()));
  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->GetSize());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->GetPos());
}

//////////////////////////////////////////////////
void HeightmapShape::ProcessMsg(const msgs::Geometry & /*_msg*/)
{
  gzerr << "TODO: not implement yet.";
}

//////////////////////////////////////////////////
math::Vector2i HeightmapShape::GetVertexCount() const
{
  return math::Vector2i(this->vertSize, this->vertSize);
}

/////////////////////////////////////////////////
float HeightmapShape::GetHeight(int _x, int _y) const
{
  return this->heights[(_y * this->subSampling) * this->vertSize +
                       (_x * this->subSampling)];
}

/////////////////////////////////////////////////
float HeightmapShape::GetMaxHeight() const
{
  float max = GZ_FLT_MIN;
  for (unsigned int i = 0; i < this->heights.size(); ++i)
  {
    if (this->heights[i] > max)
      max = this->heights[i];
  }

  return max;
}

/////////////////////////////////////////////////
float HeightmapShape::GetMinHeight() const
{
  float min = GZ_FLT_MAX;
  for (unsigned int i = 0; i < this->heights.size(); ++i)
  {
    if (this->heights[i] < min)
      min = this->heights[i];
  }

  return min;
}

//////////////////////////////////////////////////
common::Image HeightmapShape::GetImage() const
{
  double height = 0.0;
  unsigned char *imageData = NULL;

  /// \todo Support multiple terrain objects
  double minHeight = this->GetMinHeight();
  double maxHeight = this->GetMaxHeight() - minHeight;

  std::cout << "MinHeight[" << minHeight << "] MaxHeight[" << maxHeight << "]\n";

  // Get the number of vertices along one side of the terrain
  // uint16_t size = terrain->getSize();

  // Create the image data buffer
  imageData = new unsigned char[this->vertSize * this->vertSize];

  // Get height data from all vertices
  for (uint16_t y = 0; y < this->vertSize; ++y)
  {
    for (uint16_t x = 0; x < this->vertSize; ++x)
    {
      // Normalize height value
      height = (this->GetHeight((int)x, (int)y) - minHeight) / maxHeight;

      GZ_ASSERT(height <= 1.0, "Normalized terrain height > 1.0");
      GZ_ASSERT(height >= 0.0, "Normalized terrain height < 0.0");

      // Scale height to a value between 0 and 255
      imageData[y * this->vertSize + x] =
        static_cast<unsigned char>(height * 255.0);
    }
  }

  common::Image result;
  result.SetFromData(imageData, this->vertSize, this->vertSize,
                     common::Image::L_INT8);

  delete [] imageData;
  return result;
}
