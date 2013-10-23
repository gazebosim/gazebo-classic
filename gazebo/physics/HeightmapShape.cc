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
/* Desc: Heightmap shape
 * Author: Nate Koenig, Andrew Howard
 * Date: 8 May 2003
 */

#include <gazebo/gazebo_config.h>

#ifdef HAVE_GDAL
# include <gdal/gdalwarper.h>
#endif
#include <string.h>
#include <math.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
HeightmapShape::HeightmapShape(CollisionPtr _parent)
    : Shape(_parent)
{
  this->vertSize = 0;
  this->AddType(Base::HEIGHTMAP_SHAPE);
  this->fileFormat = "ANY_IMAGE";
}

//////////////////////////////////////////////////
HeightmapShape::~HeightmapShape()
{
  // ToDo: Delete objects created
}

//////////////////////////////////////////////////
void HeightmapShape::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "heightmap_data")
  {
    msgs::Geometry msg;

    msgs::Response response;
    response.set_id(_msg->id());
    response.set_request(_msg->request());
    response.set_response("success");

    this->FillMsg(msg);

    response.set_type(msg.GetTypeName());
    std::string *serializedData = response.mutable_serialized_data();
    msg.SerializeToString(serializedData);

    this->responsePub->Publish(response);
  }
}

#ifdef HAVE_GDAL
void HeightmapShape::LoadTerrainFile(std::string _filename)
{
  // Register the GDAL drivers
  GDALAllRegister();

  GDALDataset *poDataset = reinterpret_cast<GDALDataset *>
      (GDALOpen(_filename.c_str(), GA_ReadOnly));

  if (!poDataset)
    gzthrow("Unrecognized terrain format in file [" + _filename + "]\n");

  this->fileFormat = poDataset->GetDriver()->GetDescription();
  GDALClose((GDALDataset *)poDataset);

  // Check if the heightmap file is an image
  if (fileFormat == "JPEG" || fileFormat == "PNG")
  {
    // Load the terrain file as an image
    this->img.Load(_filename);
    this->heightmapData = static_cast<common::HeightmapData*>(&(this->img));
    this->heigthmapSize = this->sdf->Get<math::Vector3>("size");
  }
  else
  {
    // Load the terrain file as a SDTS
    sdts = new common::SDTS(_filename);
    this->heigthmapSize.x = sdts->GetWorldWidth();
    this->heigthmapSize.y = sdts->GetWorldHeight();
    this->heigthmapSize.z = 10.0;
    this->heightmapData = static_cast<common::HeightmapData*>(this->sdts);
  }
}
#else
void HeightmapShape::LoadTerrainFile(std::string _filename)
{
  // Load the terrain file as an image
  this->img.Load(filename);
  this->heightmapData = static_cast<common::HeightmapData*>(&(this->img));
  this->heigthmapSize = this->sdf->Get<math::Vector3>("size");
}
#endif

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::string filename = common::find_file(this->sdf->Get<std::string>("uri"));
  if (filename.empty())
  {
    gzthrow("Unable to find heightmap[" +
            this->sdf->Get<std::string>("uri") + "]\n");
  }

  LoadTerrainFile(filename);

  // Check if the geometry of the terrain data matches Ogre constrains
  if (this->heightmapData->GetWidth() != this->heightmapData->GetHeight() ||
      !math::isPowerOfTwo(this->heightmapData->GetWidth()-1))
  {
    gzthrow("Heightmap data size must be square, with a size of 2^n+1\n");
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
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->requestSub = this->node->Subscribe("~/request",
      &HeightmapShape::OnRequest, this, true);
  this->responsePub = this->node->Advertise<msgs::Response>("~/response");

  this->subSampling = 2;

  math::Vector3 terrainSize = this->GetSize();

  // sampling size along image width and height
  this->vertSize = (this->heightmapData->GetWidth() * this->subSampling)-1;
  this->scale.x = terrainSize.x / this->vertSize;
  this->scale.y = terrainSize.y / this->vertSize;

  if (math::equal(this->heightmapData->GetMaxColor().r, 0.0f))
    this->scale.z = fabs(terrainSize.z);
  else
    this->scale.z = fabs(terrainSize.z) / this->heightmapData->GetMaxColor().r;

  // Step 1: Construct the heightmap lookup table
  this->heightmapData->FillHeightMap(this->heights, this->subSampling,
    this->vertSize, this->GetSize(), this->scale, this->flipY);
}

//////////////////////////////////////////////////
void HeightmapShape::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;
}

//////////////////////////////////////////////////
std::string HeightmapShape::GetURI() const
{
  return this->sdf->Get<std::string>("uri");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetSize() const
{
  return this->heigthmapSize;
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetPos() const
{
  return this->sdf->Get<math::Vector3>("pos");
}

//////////////////////////////////////////////////
void HeightmapShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::HEIGHTMAP);

  _msg.mutable_heightmap()->set_width(this->vertSize);
  _msg.mutable_heightmap()->set_height(this->vertSize);

  for (unsigned int y = 0; y < this->vertSize; ++y)
  {
    for (unsigned int x = 0; x < this->vertSize; ++x)
    {
      int index = (this->vertSize - y - 1) * this->vertSize + x;
      _msg.mutable_heightmap()->add_heights(this->heights[index]);
    }
  }

  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->GetSize());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->GetPos());
  _msg.mutable_heightmap()->set_filename(this->img.GetFilename());
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
  if (_x < 0 || _y < 0)
    return 0.0;

  return this->heights[_y * this->vertSize + _x];
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

  int size = (this->vertSize+1) / this->subSampling;

  // Create the image data buffer
  imageData = new unsigned char[size * size];

  // Get height data from all vertices
  for (uint16_t y = 0; y < size; ++y)
  {
    for (uint16_t x = 0; x < size; ++x)
    {
      int sx = static_cast<int>(x * this->subSampling);
      int sy;

      if (!this->flipY)
        sy = static_cast<int>(y * this->subSampling);
      else
        sy = static_cast<int>(size - 1 -y) * this->subSampling;

      // Normalize height value
      height = (this->GetHeight(sx, sy) - minHeight) / maxHeight;

      GZ_ASSERT(height <= 1.0, "Normalized terrain height > 1.0");
      GZ_ASSERT(height >= 0.0, "Normalized terrain height < 0.0");

      // Scale height to a value between 0 and 255
      imageData[y * size + x] = static_cast<unsigned char>(height * 255.0);
    }
  }

  common::Image result;
  result.SetFromData(imageData, size, size, common::Image::L_INT8);

  delete [] imageData;
  return result;
}
