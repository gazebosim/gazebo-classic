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

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/PlaneShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PlaneShape::PlaneShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(PLANE_SHAPE);
  this->SetName("plane_shape");
}

//////////////////////////////////////////////////
PlaneShape::~PlaneShape()
{
}

//////////////////////////////////////////////////
void PlaneShape::Init()
{
  this->CreatePlane();
}

//////////////////////////////////////////////////
void PlaneShape::CreatePlane()
{
}

//////////////////////////////////////////////////
void PlaneShape::SetAltitude(const math::Vector3 &/*_pos*/)
{
}

//////////////////////////////////////////////////
void PlaneShape::SetNormal(const math::Vector3 &_norm)
{
  this->sdf->GetElement("normal")->Set(_norm);
  this->CreatePlane();
}

//////////////////////////////////////////////////
math::Vector3 PlaneShape::GetNormal() const
{
  return this->sdf->Get<math::Vector3>("normal");
}

//////////////////////////////////////////////////
void PlaneShape::SetSize(const math::Vector2d &_size)
{
  this->SetSize(_size.Ign());
}

//////////////////////////////////////////////////
void PlaneShape::SetSize(const ignition::math::Vector2d &_size)
{
  this->sdf->GetElement("size")->Set(_size);
}

//////////////////////////////////////////////////
math::Vector2d PlaneShape::GetSize() const
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return this->Size();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}

//////////////////////////////////////////////////
ignition::math::Vector2d PlaneShape::Size() const
{
  return this->sdf->Get<ignition::math::Vector2d>("size");
}

//////////////////////////////////////////////////
void PlaneShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;

  auto size = this->Size() * ignition::math::Vector2d(_scale.X(), scale.Y());
  this->SetSize(size);
}

//////////////////////////////////////////////////
void PlaneShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::PLANE);
  msgs::Set(_msg.mutable_plane()->mutable_normal(), this->GetNormal().Ign());
  msgs::Set(_msg.mutable_plane()->mutable_size(), this->Size());
}

//////////////////////////////////////////////////
void PlaneShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetNormal(msgs::ConvertIgn(_msg.plane().normal()));
}

//////////////////////////////////////////////////
double PlaneShape::ComputeVolume() const
{
  return 0;
}
