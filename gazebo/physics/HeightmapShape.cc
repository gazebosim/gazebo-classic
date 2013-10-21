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

#include <string.h>
#include <math.h>
#include <gdal/gdalwarper.h>

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

  // Register the GDAL drivers
  GDALAllRegister();
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

    response.set_type(msg.GetTypeName());
    std::string *serializedData = response.mutable_serialized_data();
    msg.SerializeToString(serializedData);

    this->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  std::cout << "Load()\n";
  GDALDataset *poDataset;

  Base::Load(_sdf);

  std::string filename = common::find_file(this->sdf->Get<std::string>("uri"));
  if (filename.empty())
  {
    gzthrow("Unable to find heightmap[" +
            this->sdf->Get<std::string>("uri") + "]\n");
  }

  poDataset = reinterpret_cast<GDALDataset *>
      (GDALOpen(filename.c_str(), GA_ReadOnly));
  this->fileFormat = poDataset->GetDriver()->GetDescription();
  std::cout << "Terrain file format: " << fileFormat << std::endl;
  GDALClose( (GDALDataset *) poDataset );

  // Check if the terrain file is not an image
  /*if (fileFormat != "JPEG" && fileFormat != "PNG")
  {
    // Convert to jpeg
    const char *pszFormat = "JPEG";
    GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    if (poDriver == NULL)
        gzthrow("Driver not supported\n");

    boost::filesystem::path tmpDir = boost::filesystem::temp_directory_path();
    boost::filesystem::path aDir = boost::filesystem::unique_path();
    boost::filesystem::path tmpFile = boost::filesystem::unique_path(
      "terrain-%%%%%%%%.jpg");
    boost::filesystem::path tmpPath = tmpDir / aDir / tmpFile;
    GDALDataset *poDstDS = poDriver->CreateCopy(
        tmpPath.native().c_str(), poDataset, FALSE, NULL, NULL, NULL);
    std::cout << "File generated in " << tmpPath.native() << std::endl;

    // Once we're done, close properly the dataset 
    if(poDstDS != NULL)
      GDALClose( (GDALDatasetH) poDstDS );

    filename = tmpPath.native().c_str();

    //poDataset = reinterpret_cast<GDALDataset *>
    //    (GDALOpen("output.jpg", GA_ReadOnly)); 
  }
  GDALClose( (GDALDataset *) poDataset );*/

  // Convert to jpeg
  /*GDALDataset *poDstDS;
  GDALDriver *poDriver;
  const char *pszFormat = "JPEG";
  poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
  if( poDriver == NULL )
  {
      gzthrow("Driver not supported\n");
  }

  poDstDS = poDriver->CreateCopy( "output.jpg", poDataset, FALSE, NULL, NULL, NULL );
*/
  /* Once we're done, close properly the dataset */
  //if( poDstDS != NULL )
  //  GDALClose( (GDALDatasetH) poDstDS );

  //poDataset = reinterpret_cast<GDALDataset *>
  //    (GDALOpen("output.jpg", GA_ReadOnly));

  // Check if the heightmap file is an image
  if (fileFormat == "JPEG" || fileFormat == "PNG")
  {
    // Load the terrain file as an image
    this->img.Load(filename);
    //this->img.Rescale(65, 65);
    this->heightmapData = static_cast<common::HeightmapData*>(&(this->img));
    this->heigthmapSize = this->sdf->Get<math::Vector3>("size");
  }
  else
  {
    std::cout << "SDTS\n";
    // Load the terrain file as a SDTS
    sdts = new common::SDTS(filename);
    this->heigthmapSize.x = sdts->GetWorldWidth();
    this->heigthmapSize.y = sdts->GetWorldHeight();
    this->heigthmapSize.z = 10.0;

    this->heightmapData = static_cast<common::HeightmapData*>(this->sdts);
  }

  // Check if the geometry of the terrain data matches Ogre constrains
  if (this->heightmapData->GetWidth() != this->heightmapData->GetHeight() ||
      !math::isPowerOfTwo(this->heightmapData->GetWidth()-1))
  {
    gzthrow("Heightmap data size must be square, with a size of 2^n+1\n");
  }

  std::cout << "Size: " << this->heigthmapSize << std::endl;
}

//////////////////////////////////////////////////
int HeightmapShape::GetSubSampling() const
{
  return this->subSampling;
}

//////////////////////////////////////////////////
void HeightmapShape::Init()
{
  std::cout << "Init()\n";
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

  std::cout << "Physics. GetSize() GetSize: " << this->GetSize() << std::endl;
  std::cout << "Physics. Init() GetWidth: " << this->heightmapData->GetWidth() << std::endl;
  std::cout << "Physics. Init() vertSize: " << this->vertSize << std::endl;

  if (math::equal(this->heightmapData->GetMaxColor().r, 0.0f))
    this->scale.z = fabs(terrainSize.z);
  else
    this->scale.z = fabs(terrainSize.z) / this->heightmapData->GetMaxColor().r;

  // Step 1: Construct the heightmap lookup table
  this->FillHeightMap();
}

//////////////////////////////////////////////////
void HeightmapShape::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;
}

//////////////////////////////////////////////////
void HeightmapShape::FillHeightMap()
{
  unsigned int x, y;
  float h = 0;
  float h1 = 0;
  float h2 = 0;

  // Resize the vector to match the size of the vertices.
  this->heights.resize(this->vertSize * this->vertSize);

  common::Color pixel;

  int imgHeight = this->heightmapData->GetHeight();
  int imgWidth = this->heightmapData->GetWidth();

  GZ_ASSERT(imgWidth == imgHeight, "Heightmap image must be square");

  // Bytes per row
  unsigned int pitch = this->heightmapData->GetPitch();

  // Bytes per pixel
  unsigned int bpp = pitch / imgWidth;

  unsigned char *data = NULL;
  unsigned int count;
  this->heightmapData->GetData(&data, count);

  double yf, xf, dy, dx;
  int y1, y2, x1, x2;
  double px1, px2, px3, px4;

  // Iterate over all the vertices
  for (y = 0; y < this->vertSize; ++y)
  {
    // yf ranges between 0 and 4
    yf = y / static_cast<double>(this->subSampling);
    y1 = floor(yf);
    y2 = ceil(yf);
    if (y2 >= imgHeight)
      y2 = imgHeight-1;
    dy = yf - y1;

    for (x = 0; x < this->vertSize; ++x)
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
      if (!this->flipY)
        this->heights[y * this->vertSize + x] = h;
      else
        this->heights[(this->vertSize - y - 1) * this->vertSize + x] = h;
    }
  }

  delete [] data;
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

  std::cout << "Physics. FillMsg() GetSize: " << this->GetSize() << std::endl;
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
