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
#include "gazebo/physics/World.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/physics/HeightmapShapePrivate.hh"
#include "gazebo/physics/HeightmapShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
HeightmapShape::HeightmapShape(CollisionPtr _parent)
: Shape(*new HeightmapShapePrivate, _parent),
  heightmapShapeDPtr(static_cast<HeightmapShapePrivate*>(this->shapeDPtr))
{
  this->heightmapShapeDPtr = 0;
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

    this->heightmapShapeDPtr->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
int HeightmapShape::LoadTerrainFile(const std::string &_filename)
{
  this->heightmapShapeDPtr->heightmapData =
    common::HeightmapDataLoader::LoadTerrainFile(_filename);

  if (!this->heightmapShapeDPtr->heightmapData)
  {
    gzerr << "Unable to load heightmap data" << std::endl;
    return -1;
  }

#ifdef HAVE_GDAL
  // DEM
  auto demData = dynamic_cast<common::Dem *>(
      this->heightmapShapeDPtr->heightmapData);

  if (demData)
  {
    this->heightmapShapeDPtr->dem = *demData;
    if (this->heightmapShapeDPtr->sdf->HasElement("size"))
    {
      this->heightmapShapeDPtr->heightmapSize =
        this->heightmapShapeDPtr->sdf->Get<ignition::math::Vector3d>("size");
    }
    else
    {
      this->heightmapShapeDPtr->heightmapSize.X(
        this->heightmapShapeDPtr->dem.GetWorldWidth());
      this->heightmapShapeDPtr->heightmapSize.Y(
        this->heightmapShapeDPtr->dem.GetWorldHeight());
      this->heightmapShapeDPtr->heightmapSize.Z(
        this->heightmapShapeDPtr->dem.GetMaxElevation() -
          std::max(0.0f, this->heightmapShapeDPtr->dem.GetMinElevation()));
    }

    // Modify the reference geotedic latitude/longitude.
    // A GPS sensor will use the real georeferenced coordinates of the terrain.
    common::SphericalCoordinatesPtr sphericalCoordinates =
      this->heightmapShapeDPtr->world->SphericalCoords();

    if (sphericalCoordinates)
    {
      ignition::math::Angle latitude, longitude;
      double elevation;

      this->heightmapShapeDPtr->dem.GetGeoReferenceOrigin(latitude, longitude);
      elevation = this->heightmapShapeDPtr->dem.GetElevation(0.0, 0.0);

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
        dynamic_cast<common::ImageHeightmap *>(
            this->heightmapShapeDPtr->heightmapData);
    if (imageData)
    {
      this->heightmapShapeDPtr->img = *imageData;
      this->heightmapShapeDPtr->heightmapSize =
        this->heightmapShapeDPtr->sdf->Get<ignition::math::Vector3d>("size");
      return 0;
    }
  }

  return -1;
}

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::string filename = common::find_file(
      this->heightmapShapeDPtr->sdf->Get<std::string>("uri"));
  if (filename.empty())
  {
    gzerr << "Unable to find heightmap[" +
             this->heightmapShapeDPtr->sdf->Get<std::string>("uri") + "]\n";
    return;
  }

  if (this->LoadTerrainFile(filename) != 0)
  {
    gzerr << "Heightmap data size must be square, with a size of 2^n+1\n";
    return;
  }

  // Check if the geometry of the terrain data matches Ogre constrains
  if (this->heightmapShapeDPtr->heightmapData->GetWidth() !=
      this->heightmapShapeDPtr->heightmapData->GetHeight() ||
      !ignition::math::isPowerOfTwo(
        this->heightmapShapeDPtr->heightmapData->GetWidth() - 1))
  {
    gzerr << "Heightmap data size must be square, with a size of 2^n+1\n";
    return;
  }
}

//////////////////////////////////////////////////
int HeightmapShape::GetSubSampling() const
{
  return this->SubSampling();
}

//////////////////////////////////////////////////
int HeightmapShape::SubSampling() const
{
  return this->heightmapShapeDPtr->subSampling;
}

//////////////////////////////////////////////////
void HeightmapShape::Init()
{
  this->heightmapShapeDPtr->node = transport::NodePtr(new transport::Node());
  this->heightmapShapeDPtr->node->Init();

  this->heightmapShapeDPtr->requestSub =
    this->heightmapShapeDPtr->node->Subscribe("~/request",
      &HeightmapShape::OnRequest, this, true);

  this->heightmapShapeDPtr->responsePub =
    this->heightmapShapeDPtr->node->Advertise<msgs::Response>("~/response");

  this->heightmapShapeDPtr->subSampling = 2;

  ignition::math::Vector3d terrainSize = this->Size();

  // sampling size along image width and height
  this->heightmapShapeDPtr->vertSize = (
      this->heightmapShapeDPtr->heightmapData->GetWidth() *
      this->heightmapShapeDPtr->subSampling)-1;

  this->heightmapShapeDPtr->scale.X(
      terrainSize.X() / this->heightmapShapeDPtr->vertSize);
  this->heightmapShapeDPtr->scale.Y(
      terrainSize.Y() / this->heightmapShapeDPtr->vertSize);

  if (ignition::math::equal(
        this->heightmapShapeDPtr->heightmapData->GetMaxElevation(), 0.0f))
  {
    this->heightmapShapeDPtr->scale.Z(fabs(terrainSize.Z()));
  }
  else
  {
    this->heightmapShapeDPtr->scale.Z(fabs(terrainSize.Z()) /
                  this->heightmapShapeDPtr->heightmapData->GetMaxElevation());
  }

  // Step 1: Construct the heightmap lookup table
  this->heightmapShapeDPtr->heightmapData->FillHeightMap(
      this->heightmapShapeDPtr->subSampling, this->heightmapShapeDPtr->vertSize,
      this->Size(), this->heightmapShapeDPtr->scale,
      this->heightmapShapeDPtr->flipY,
      this->heightmapShapeDPtr->heights);
}

//////////////////////////////////////////////////
void HeightmapShape::SetScale(const math::Vector3 &_scale)
{
  this->SetScale(_scale.Ign());
}

//////////////////////////////////////////////////
void HeightmapShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (this->heightmapShapeDPtr->scale == _scale)
    return;

  this->heightmapShapeDPtr->scale = _scale;
}

//////////////////////////////////////////////////
std::string HeightmapShape::GetURI() const
{
  return this->URI();
}

//////////////////////////////////////////////////
std::string HeightmapShape::URI() const
{
  return this->heightmapShapeDPtr->sdf->Get<std::string>("uri");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetSize() const
{
  return this->Size();
}

//////////////////////////////////////////////////
ignition::math::Vector3d HeightmapShape::Size() const
{
  return this->heightmapShapeDPtr->heightmapSize;
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetPos() const
{
  return this->Pos();
}

//////////////////////////////////////////////////
ignition::math::Vector3d HeightmapShape::Pos() const
{
  return this->heightmapShapeDPtr->sdf->Get<ignition::math::Vector3d>("pos");
}

//////////////////////////////////////////////////
void HeightmapShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::HEIGHTMAP);

  _msg.mutable_heightmap()->set_width(this->heightmapShapeDPtr->vertSize);
  _msg.mutable_heightmap()->set_height(this->heightmapShapeDPtr->vertSize);

  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->Size());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->Pos());
  _msg.mutable_heightmap()->set_filename(this->URI());
}

//////////////////////////////////////////////////
void HeightmapShape::FillHeights(msgs::Geometry &_msg) const
{
  for (unsigned int y = 0; y < this->heightmapShapeDPtr->vertSize; ++y)
  {
    for (unsigned int x = 0; x < this->heightmapShapeDPtr->vertSize; ++x)
    {
      int index = (this->heightmapShapeDPtr->vertSize - y - 1) *
        this->heightmapShapeDPtr->vertSize + x;
      _msg.mutable_heightmap()->add_heights(
          this->heightmapShapeDPtr->heights[index]);
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
  return this->VertexCount();
}

//////////////////////////////////////////////////
ignition::math::Vector2i HeightmapShape::VertexCount() const
{
  return ignition::math::Vector2i(this->heightmapShapeDPtr->vertSize,
      this->heightmapShapeDPtr->vertSize);
}

/////////////////////////////////////////////////
float HeightmapShape::GetHeight(int _x, int _y) const
{
  return this->Height(_x, _y);
}

/////////////////////////////////////////////////
float HeightmapShape::Height(const int _x, const int _y) const
{
  int index = _y * this->heightmapShapeDPtr->vertSize + _x;
  if (_x < 0 || _y < 0 ||
      index >= static_cast<int>(this->heightmapShapeDPtr->heights.size()))
    return 0.0;

  return this->heightmapShapeDPtr->heights[index];
}

/////////////////////////////////////////////////
float HeightmapShape::GetMaxHeight() const
{
  return this->MaxHeight();
}

/////////////////////////////////////////////////
float HeightmapShape::MaxHeight() const
{
  float max = IGN_FLT_MIN;
  for (unsigned int i = 0; i < this->heightmapShapeDPtr->heights.size(); ++i)
  {
    if (this->heightmapShapeDPtr->heights[i] > max)
      max = this->heightmapShapeDPtr->heights[i];
  }

  return max;
}

/////////////////////////////////////////////////
float HeightmapShape::GetMinHeight() const
{
  return this->MinHeight();
}

/////////////////////////////////////////////////
float HeightmapShape::MinHeight() const
{
  float min = IGN_FLT_MAX;
  for (unsigned int i = 0; i < this->heightmapShapeDPtr->heights.size(); ++i)
  {
    if (this->heightmapShapeDPtr->heights[i] < min)
      min = this->heightmapShapeDPtr->heights[i];
  }

  return min;
}

//////////////////////////////////////////////////
common::Image HeightmapShape::GetImage() const
{
  return this->Image();
}

//////////////////////////////////////////////////
common::Image HeightmapShape::Image() const
{
  double height = 0.0;
  unsigned char *imageData = NULL;

  /// \todo Support multiple terrain objects
  double minHeight = this->MinHeight();
  double maxHeight = this->MaxHeight() - minHeight;

  int size = (this->heightmapShapeDPtr->vertSize+1) /
    this->heightmapShapeDPtr->subSampling;

  // Create the image data buffer
  imageData = new unsigned char[size * size];

  // Get height data from all vertices
  for (uint16_t y = 0; y < size; ++y)
  {
    for (uint16_t x = 0; x < size; ++x)
    {
      int sx = static_cast<int>(x * this->heightmapShapeDPtr->subSampling);
      int sy;

      if (!this->heightmapShapeDPtr->flipY)
      {
        sy = static_cast<int>(y * this->heightmapShapeDPtr->subSampling);
      }
      else
      {
        sy = static_cast<int>(size - 1 -y) *
          this->heightmapShapeDPtr->subSampling;
      }

      // Normalize height value
      height = (this->Height(sx, sy) - minHeight) / maxHeight;

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
