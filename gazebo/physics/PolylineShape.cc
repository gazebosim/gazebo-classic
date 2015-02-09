/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
      meshName, this->GetVertices(), this->GetHeight());

  this->mesh = common::MeshManager::Instance()->GetMesh(meshName);

  if (!this->mesh)
    gzerr << "Unable to create polyline mesh\n";
}

//////////////////////////////////////////////////
std::vector<std::vector<math::Vector2d> > PolylineShape::GetVertices() const
{
  std::vector<std::vector<math::Vector2d> > paths;

  sdf::ElementPtr polylineElem = this->sdf;
  while (polylineElem)
  {
    sdf::ElementPtr pointElem = polylineElem->GetElement("point");
    std::vector<math::Vector2d> vertices;
    while (pointElem)
    {
      math::Vector2d point = pointElem->Get<math::Vector2d>();
      pointElem = pointElem->GetNextElement("point");
      vertices.push_back(point);
    }
    polylineElem = polylineElem->GetNextElement("polyline");
    paths.push_back(vertices);
  }
  return paths;
}

//////////////////////////////////////////////////
void PolylineShape::SetHeight(const double &_height)
{
  sdf::ElementPtr polylineElem = this->sdf;
  while (polylineElem)
  {
    polylineElem->GetElement("height")->Set(_height);
    polylineElem = polylineElem->GetNextElement("polyline");
  }
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
  sdf::ElementPtr geomSDF = msgs::GeometryToSDF(_msg);
  this->sdf = geomSDF->GetElement("polyline");
}

////////////////////////////////////////////////////
void PolylineShape::SetVertices(
    const std::vector<std::vector<math::Vector2d> > &_vertices)
{
  sdf::ElementPtr polylineElem = this->sdf;
  for (unsigned int i = 0; i < _vertices.size(); ++i)
  {
    std::vector<math::Vector2d> v = _vertices[i];

    sdf::ElementPtr pointElem = polylineElem->GetElement("point");
    for (unsigned int j = 0; j < v.size(); ++j)
    {
      pointElem->Set(v[j]);
      pointElem = pointElem->GetNextElement("point");
    }
    polylineElem = polylineElem->GetNextElement("polyline");
  }
}

//////////////////////////////////////////////////
void PolylineShape::SetPolylineShape(const double &_height,
    const std::vector<std::vector<math::Vector2d> > &_vertices)
{
  this->SetHeight(_height);
  this->SetVertices(_vertices);
}

//////////////////////////////////////////////////
void PolylineShape::FillMsg(msgs::Geometry &_msg)
{
  _msg = msgs::GeometryFromSDF(this->sdf->GetParent());
}

//////////////////////////////////////////////////
void PolylineShape::ProcessMsg(const msgs::Geometry &_msg)
{
  if (_msg.polyline_size() > 0)
  {
    this->SetHeight(_msg.polyline(0).height());
    this->SetVertices(_msg);
  }
  else
    gzerr << "Unable to process message, no polyline shape.\n";
}
