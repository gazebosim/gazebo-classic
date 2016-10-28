/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <algorithm>
#include <cmath>
#include <string>
#include <ignition/math/Helpers.hh>
#include <gazebo/gazebo_config.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
HeightmapShape::HeightmapShape(CollisionPtr _parent)
    : Shape(_parent)
{
  this->vertSize = 0;
  this->AddType(Base::HEIGHTMAP_SHAPE);
}

//////////////////////////////////////////////////
HeightmapShape::~HeightmapShape()
{
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
    this->FillHeights(msg);

    response.set_type(msg.GetTypeName());
    std::string *serializedData = response.mutable_serialized_data();
    msg.SerializeToString(serializedData);

    this->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
int HeightmapShape::LoadTerrainFile(const std::string &_filename)
{
  this->heightmapData = common::HeightmapDataLoader::LoadTerrainFile(_filename);
  if (!this->heightmapData)
  {
    gzerr << "Unable to load heightmap data" << std::endl;
    return -1;
  }

#ifdef HAVE_GDAL
  // DEM
  auto demData = dynamic_cast<common::Dem *>(this->heightmapData);
  if (demData)
  {
    this->dem = *demData;
    if (this->sdf->HasElement("size"))
    {
      this->heightmapSize = this->sdf->Get<math::Vector3>("size");
    }
    else
    {
      this->heightmapSize.x = this->dem.GetWorldWidth();
      this->heightmapSize.y = this->dem.GetWorldHeight();
      this->heightmapSize.z = this->dem.GetMaxElevation() -
          this->dem.GetMinElevation();
    }

    // Modify the reference geotedic latitude/longitude.
    // A GPS sensor will use the real georeferenced coordinates of the terrain.
    common::SphericalCoordinatesPtr sphericalCoordinates;
    sphericalCoordinates = this->world->GetSphericalCoordinates();

    if (sphericalCoordinates)
    {
      ignition::math::Angle latitude, longitude;
      double elevation;

      this->dem.GetGeoReferenceOrigin(latitude, longitude);
      elevation = this->dem.GetElevation(0.0, 0.0);

      sphericalCoordinates->SetLatitudeReference(latitude);
      sphericalCoordinates->SetLongitudeReference(longitude);
      sphericalCoordinates->SetElevationReference(elevation);
      sphericalCoordinates.reset();
    }
    else
      gzerr << "Unable to get a valid SphericalCoordinates pointer\n";

    return 0;
  }
  // Image
  else
#endif
  {
    auto imageData =
        dynamic_cast<common::ImageHeightmap *>(this->heightmapData);
    if (imageData)
    {
      this->img = *imageData;
      this->heightmapSize = this->sdf->Get<math::Vector3>("size");
      return 0;
    }
  }

  return -1;
}

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::string filename = common::find_file(this->sdf->Get<std::string>("uri"));
  if (filename.empty())
  {
    gzerr << "Unable to find heightmap[" +
             this->sdf->Get<std::string>("uri") + "]\n";
    return;
  }

  if (this->LoadTerrainFile(filename) != 0)
  {
    gzerr << "Heightmap data size must be square, with a size of 2^n+1\n";
    return;
  }

  // Check if the geometry of the terrain data matches Ogre constrains
  if (this->heightmapData->GetWidth() != this->heightmapData->GetHeight() ||
      !ignition::math::isPowerOfTwo(this->heightmapData->GetWidth() - 1))
  {
    gzerr << "Heightmap data size must be square, with a size of 2^n+1\n";
    return;
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

  // TODO add a virtual HeightmapData::GetMinElevation function to avoid the
  // ifdef check. i.e. heightmapSizeZ = GetMaxElevation - GetMinElevation
  double heightmapSizeZ = this->heightmapData->GetMaxElevation();
#ifdef HAVE_GDAL
  // DEM
  auto demData = dynamic_cast<common::Dem *>(this->heightmapData);
  if (demData)
    heightmapSizeZ = heightmapSizeZ - demData->GetMinElevation();
#endif

  if (ignition::math::equal(heightmapSizeZ, 0.0))
    this->scale.z = 1.0;
  else
    this->scale.z = fabs(terrainSize.z) / heightmapSizeZ;

  // Step 1: Construct the heightmap lookup table
  this->heightmapData->FillHeightMap(this->subSampling, this->vertSize,
      this->GetSize().Ign(), this->scale.Ign(), this->flipY, this->heights);
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
  return this->heightmapSize;
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

  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->GetSize().Ign());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->GetPos().Ign());
  _msg.mutable_heightmap()->set_filename(this->GetURI());
}

//////////////////////////////////////////////////
void HeightmapShape::FillHeights(msgs::Geometry &_msg) const
{
  for (unsigned int y = 0; y < this->vertSize; ++y)
  {
    for (unsigned int x = 0; x < this->vertSize; ++x)
    {
      int index = (this->vertSize - y - 1) * this->vertSize + x;
      _msg.mutable_heightmap()->add_heights(this->heights[index]);
    }
  }
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
  int index =  _y * this->vertSize + _x;
  if (_x < 0 || _y < 0 || index >= static_cast<int>(this->heights.size()))
    return 0.0;

  return this->heights[index];
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

//////////////////////////////////////////////////
double HeightmapShape::ComputeVolume() const
{
  return 0;
}
