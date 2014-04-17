/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <vector>
#include <stdio.h>
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/PolyLineShape.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Console.hh"
using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PolyLineShape::PolyLineShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::POLYLINE_SHAPE);
  this->scale = math::Vector3::One;
  sdf::initFile("polyline_shape.sdf", this->sdf);
  this->Init();
}

//////////////////////////////////////////////////
PolyLineShape::~PolyLineShape()
{
}

//////////////////////////////////////////////////
void PolyLineShape::Init()
{
  this->SetPolylineShape(this->GetHeight(),
                         this->GetVertices());
  common::MeshManager *meshManager = common::MeshManager::Instance();

  meshManager->CreateExtrudedPolyline("extruded_polyline", this->GetVertices(),
                                      this->GetHeight(), math::Vector2d(1, 1));
}

//////////////////////////////////////////////////
std::vector<math::Vector2d> PolyLineShape::GetVertices() const
{
  std::vector<math::Vector2d> vertices;
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  while (pointElem)
  {
     math::Vector2d point = pointElem->Get<math::Vector2d>();
     pointElem = pointElem->GetNextElement("point");
     vertices.push_back(point);
  }
  return vertices;
}

//////////////////////////////////////////////////
void PolyLineShape::SetHeight(const double &_height)
{
  this->sdf->GetElement("height")->Set(_height);
}

//////////////////////////////////////////////////
double PolyLineShape::GetHeight() const
{
  return this->sdf->Get<double>("height");
}

//////////////////////////////////////////////////
void PolyLineShape::SetScale(const math::Vector3 &_scale)
{
  if (_scale.x < 0 || _scale.y < 0 || _scale.z < 0)
    return;

  if (_scale == this->scale)
    return;


  this->scale = _scale;
}

////////////////////////////////////////////////////
void PolyLineShape::SetVertices(const msgs::Geometry &_msg)
{
  int i;
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  for (i = 0; i < _msg.polyline().point_size(); i++)
  {
    math::Vector2d point(_msg.polyline().point(i).x(),
                         _msg.polyline().point(i).y());
    pointElem->Set(point);
    pointElem = pointElem->GetNextElement("point");
  }
}

////////////////////////////////////////////////////
void PolyLineShape::SetVertices(const std::vector<math::Vector2d> &_vertices)
{
  unsigned int i;
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  for (i = 0; i < _vertices.size(); i++)
  {
    pointElem->Set(_vertices[i]);
    pointElem = pointElem->GetNextElement("point");
  }
}

//////////////////////////////////////////////////
void PolyLineShape::SetPolylineShape(const double &_height,
                                     const std::vector<math::Vector2d>
                                           &_vertices)
{
  this->SetHeight(_height);
  this->SetVertices(_vertices);
}

//////////////////////////////////////////////////
void PolyLineShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::POLYLINE);
  _msg.mutable_polyline()->set_height(this->GetHeight());
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  while (pointElem)
  {
     math::Vector2d point = pointElem->Get<math::Vector2d>();
     pointElem = pointElem->GetNextElement("point");
     msgs::Vector2d *ptMsg = _msg.mutable_polyline()->add_point();
     msgs::Set(ptMsg, point);
  }
}

//////////////////////////////////////////////////
void PolyLineShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetHeight(_msg.polyline().height());
  this->SetVertices(_msg);
}
