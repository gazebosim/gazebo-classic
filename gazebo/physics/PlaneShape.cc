/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
  this->rml.mutable_geometry()->mutable_plane_shape()->set_normal(
      sdf::Vector3(_norm.x, _norm.y, _norm.z));
  this->CreatePlane();
}

//////////////////////////////////////////////////
math::Vector3 PlaneShape::GetNormal() const
{
  return this->rml.geometry().plane_shape().normal();
}

//////////////////////////////////////////////////
void PlaneShape::SetSize(const math::Vector2d &_size)
{
  this->rml.mutable_geometry()->mutable_plane_shape()->set_size(
      sdf::Vector2d(_size.x, _size.y));
}

//////////////////////////////////////////////////
math::Vector2d PlaneShape::GetSize() const
{
  return math::Vector2d(this->rml.geometry().plane_shape().size().x,
      this->rml.geometry().plane_shape().size().y);
}

//////////////////////////////////////////////////
void PlaneShape::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;

  math::Vector2d size = this->GetSize() * math::Vector2d(_scale.x, scale.y);
  this->SetSize(size);
}

//////////////////////////////////////////////////
void PlaneShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::PLANE);
  msgs::Set(_msg.mutable_plane()->mutable_normal(), this->GetNormal());
  msgs::Set(_msg.mutable_plane()->mutable_size(), this->GetSize());
}

//////////////////////////////////////////////////
void PlaneShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetNormal(msgs::Convert(_msg.plane().normal()));
}
