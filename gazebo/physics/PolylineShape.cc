/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/PolylineShape.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PolylineShape::PolylineShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::POLYLINE_SHAPE);
  this->scale = math::Vector3::One;
  sdf::initFile("polyline_shape.sdf", this->sdf);
}

//////////////////////////////////////////////////
PolylineShape::~PolylineShape()
{
}

//////////////////////////////////////////////////
void PolylineShape::Init()
{
  this->SetPolylineShape(this->GetHeight(),
      this->GetVertices());

  std::string meshName =
    boost::lexical_cast<std::string>(physics::getUniqueId()) +
    "_extruded_polyline";

  common::MeshManager::Instance()->CreateExtrudedPolyline(
      meshName, this->GetVertices(),
      this->GetHeight(), math::Vector2d(1, 1));

  this->mesh = common::MeshManager::Instance()->GetMesh(meshName);

  if (!this->mesh)
    gzerr << "Unable to create polyline mesh\n";
}

//////////////////////////////////////////////////
std::vector<math::Vector2d> PolylineShape::GetVertices() const
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
void PolylineShape::SetHeight(const double &_height)
{
  this->sdf->GetElement("height")->Set(_height);
}

//////////////////////////////////////////////////
double PolylineShape::GetHeight() const
{
  return this->sdf->Get<double>("height");
}

//////////////////////////////////////////////////
void PolylineShape::SetScale(const math::Vector3 &_scale)
{
  if (_scale.x < 0 || _scale.y < 0 || _scale.z < 0)
    return;

  if (_scale == this->scale)
    return;

  this->scale = _scale;
}

////////////////////////////////////////////////////
void PolylineShape::SetVertices(const msgs::Geometry &_msg)
{
  if (!_msg.has_polyline())
  {
    gzerr << "Unable to set vertices from message, no polyline shape.\n";
    return;
  }

  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  for (int i = 0; i < _msg.polyline().point_size(); ++i)
  {
    math::Vector2d point(_msg.polyline().point(i).x(),
        _msg.polyline().point(i).y());
    pointElem->Set(point);
    pointElem = pointElem->GetNextElement("point");
  }
}

////////////////////////////////////////////////////
void PolylineShape::SetVertices(const std::vector<math::Vector2d> &_vertices)
{
  unsigned int i;
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  for (i = 0; i < _vertices.size(); ++i)
  {
    pointElem->Set(_vertices[i]);
    pointElem = pointElem->GetNextElement("point");
  }
}

//////////////////////////////////////////////////
void PolylineShape::SetPolylineShape(const double &_height,
    const std::vector<math::Vector2d> &_vertices)
{
  this->SetHeight(_height);
  this->SetVertices(_vertices);
}

//////////////////////////////////////////////////
void PolylineShape::FillMsg(msgs::Geometry &_msg)
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
void PolylineShape::ProcessMsg(const msgs::Geometry &_msg)
{
  if (_msg.has_polyline())
  {
    this->SetHeight(_msg.polyline().height());
    this->SetVertices(_msg);
  }
  else
    gzerr << "Unable to process message, no polyline shape.\n";
}
