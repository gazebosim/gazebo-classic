/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <cmath>
#include <string>
#include <ignition/math/Helpers.hh>
#include <gazebo/gazebo_config.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
HeightmapShape::HeightmapShape(CollisionPtr _parent)
    : Shape(_parent)
{
  static_assert(
      std::is_same<HeightType, float>::value ||
      std::is_same<HeightType, double>::value,
      "Height field needs to be double or float");
  this->vertSize = 0;
  this->AddType(Base::HEIGHTMAP_SHAPE);
}

//////////////////////////////////////////////////
HeightmapShape::~HeightmapShape()
{
  this->requestSub.reset();
  this->responsePub.reset();
  if (this->node)
    this->node->Fini();
  this->node.reset();
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
  common::SphericalCoordinatesPtr sphericalCoordinates;
  sphericalCoordinates = this->world->SphericalCoords();

  this->heightmapData = common::HeightmapDataLoader::LoadTerrainFile(_filename,
      sphericalCoordinates);
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
      this->heightmapSize = this->sdf->Get<ignition::math::Vector3d>("size");
    }
    else
    {
      this->heightmapSize.X() = this->dem.GetWorldWidth();
      this->heightmapSize.Y() = this->dem.GetWorldHeight();
      this->heightmapSize.Z() = this->dem.GetMaxElevation() -
          this->dem.GetMinElevation();
    }

    // Modify the reference geotedic latitude/longitude.
    // A GPS sensor will use the real georeferenced coordinates of the terrain.

    if (sphericalCoordinates)
    {
      ignition::math::Angle latitude, longitude;
      double elevation;

      try
      {
        this->dem.GetGeoReferenceOrigin(latitude, longitude);
      }
      catch(const common::Exception &)
      {
        gzwarn << "DEM coordinate transformation error. "
               << "SphericalCoordiantes and GpsSensor may not function properly."
               << std::endl;
      }
      elevation = this->dem.GetElevation(0.0, 0.0);

      sphericalCoordinates->SetLatitudeReference(latitude);
      sphericalCoordinates->SetLongitudeReference(longitude);
      sphericalCoordinates->SetElevationReference(elevation);
      sphericalCoordinates.reset();
    }
    else
    {
      gzerr << "Unable to get a valid SphericalCoordinates pointer\n";
    }

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
      this->heightmapSize = this->sdf->Get<ignition::math::Vector3d>("size");
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

  this->subSampling = 2u;
  if (this->sdf->HasElement("sampling"))
  {
    unsigned int s = this->sdf->Get<unsigned int>("sampling");
    if (s == 0u || s & (s - 1u))
    {
      gzerr << "Heightmap sampling value must be a power of 2. "
            << "The default value of 2 will be used instead." << std::endl;
      this->subSampling = 2;
    }
    else
    {
      this->subSampling = static_cast<int>(s);
    }
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
void HeightmapShape::FillHeightfield(std::vector<float>& _heights)
{
  this->heightmapData->FillHeightMap(this->subSampling, this->vertSize,
      this->Size(), this->scale, this->flipY, _heights);
}

//////////////////////////////////////////////////
void HeightmapShape::FillHeightfield(std::vector<double>& _heights)
{
  std::vector<float> fHeights;
  this->heightmapData->FillHeightMap(this->subSampling, this->vertSize,
      this->Size(), this->scale, this->flipY, fHeights);
  _heights = std::vector<double>(fHeights.begin(), fHeights.end());
}

//////////////////////////////////////////////////
void HeightmapShape::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->requestSub = this->node->Subscribe("~/request",
      &HeightmapShape::OnRequest, this, true);
  this->responsePub = this->node->Advertise<msgs::Response>("~/response");

  ignition::math::Vector3d terrainSize = this->Size();

  // sampling size along image width and height
  this->vertSize = (this->heightmapData->GetWidth() * this->subSampling)
      - this->subSampling + 1;
  this->scale.X() = terrainSize.X() / this->vertSize;
  this->scale.Y() = terrainSize.Y() / this->vertSize;

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
    this->scale.Z() = 1.0;
  else
    this->scale.Z() = fabs(terrainSize.Z()) / heightmapSizeZ;

  // Construct the heightmap lookup table
  this->FillHeightfield(this->heights);
}

//////////////////////////////////////////////////
void HeightmapShape::SetScale(const ignition::math::Vector3d &_scale)
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
ignition::math::Vector3d HeightmapShape::Size() const
{
  return this->heightmapSize;
}

//////////////////////////////////////////////////
ignition::math::Vector3d HeightmapShape::Pos() const
{
  return this->sdf->Get<ignition::math::Vector3d>("pos");
}

//////////////////////////////////////////////////
void HeightmapShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::HEIGHTMAP);

  _msg.mutable_heightmap()->set_width(this->vertSize);
  _msg.mutable_heightmap()->set_height(this->vertSize);

  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->Size());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->Pos());
  _msg.mutable_heightmap()->set_filename(this->GetURI());
  _msg.mutable_heightmap()->set_sampling(
      static_cast<unsigned int>(this->subSampling));
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
ignition::math::Vector2i HeightmapShape::VertexCount() const
{
  return ignition::math::Vector2i(this->vertSize, this->vertSize);
}

/////////////////////////////////////////////////
HeightmapShape::HeightType HeightmapShape::GetHeight(int _x, int _y) const
{
  int index =  _y * this->vertSize + _x;
  if (_x < 0 || _y < 0 || index >= static_cast<int>(this->heights.size()))
    return 0.0;

  return this->heights[index];
}

/////////////////////////////////////////////////
void HeightmapShape::SetHeight(int _x, int _y, HeightmapShape::HeightType _h)
{
  int index =  _y * this->vertSize + _x;
  if (_x < 0 || _y < 0 || index >= static_cast<int>(this->heights.size()))
  {
    gzerr << "SetHeight position (" << _x << ", " << _y << ")"
          << " is out of bounds" << std::endl;
    return;
  }

  this->heights[index] = _h;
}

/////////////////////////////////////////////////
HeightmapShape::HeightType HeightmapShape::GetMaxHeight() const
{
  HeightType max = -std::numeric_limits<HeightType>::max();
  for (unsigned int i = 0; i < this->heights.size(); ++i)
  {
    if (this->heights[i] > max)
      max = this->heights[i];
  }

  return max;
}

/////////////////////////////////////////////////
HeightmapShape::HeightType HeightmapShape::GetMinHeight() const
{
  HeightType min = std::numeric_limits<HeightType>::max();
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
