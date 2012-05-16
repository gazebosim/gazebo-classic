/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 */

#include <string.h>
#include <math.h>

#include "common/Image.hh"
#include "common/Exception.hh"

#include "physics/HeightmapShape.hh"

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
void HeightmapShape::Update()
{
}

//////////////////////////////////////////////////
void HeightmapShape::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  // Use the image to get the size of the heightmap
  this->img.Load(this->sdf->GetValueString("filename"));

  if (this->img.GetWidth() != this->img.GetHeight() ||
      !math::isPowerOfTwo(this->img.GetWidth()-1))
  {
    gzthrow("Heightmap image size must be square, with a size of 2^n-1\n");
  }
}

//////////////////////////////////////////////////
void HeightmapShape::Init()
{
}

//////////////////////////////////////////////////
std::string HeightmapShape::GetFilename() const
{
  return this->sdf->GetValueString("filename");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetSize() const
{
  return this->sdf->GetValueVector3("size");
}

//////////////////////////////////////////////////
math::Vector3 HeightmapShape::GetOrigin() const
{
  return this->sdf->GetValueVector3("origin");
}

//////////////////////////////////////////////////
void HeightmapShape::FillShapeMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::HEIGHTMAP);
  msgs::Set(_msg.mutable_heightmap()->mutable_image(),
            common::Image(this->GetFilename()));
  msgs::Set(_msg.mutable_heightmap()->mutable_size(), this->GetSize());
  msgs::Set(_msg.mutable_heightmap()->mutable_origin(), this->GetOrigin());
}

//////////////////////////////////////////////////
void HeightmapShape::ProcessMsg(const msgs::Geometry & /*_msg*/)
{
  gzerr << "TODO: not implement yet.";
}

//////////////////////////////////////////////////
math::Vector2i HeightmapShape::GetVertexCount() const
{
  return math::Vector2i(this->img.GetWidth(), this->img.GetHeight());
}
