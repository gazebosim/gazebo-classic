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

#include "gazebo/common/Console.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/MultiRayShapePrivate.hh"
#include "gazebo/physics/MultiRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
MultiRayShape::MultiRayShape(CollisionPtr _parent)
: Shape(*new MultiRayShapePrivate, _parent),
  multiRayShapeDPtr(static_cast<MultiRayShapePrivate*>(this->shapeDPtr))
{
  this->AddType(MULTIRAY_SHAPE);
  this->SetName("multiray");
}

//////////////////////////////////////////////////
MultiRayShape::MultiRayShape(PhysicsEnginePtr /*_physicsEngine*/)
: MultiRayShape(CollisionPtr())
{
}

//////////////////////////////////////////////////
MultiRayShape::~MultiRayShape()
{
  this->multiRayShapeDPtr->rays.clear();
}

//////////////////////////////////////////////////
void MultiRayShape::Init()
{
  ignition::math::Vector3d start, end, axis;
  double yawAngle, pitchAngle;
  ignition::math::Quaterniond ray;
  double yDiff;
  double horzMinAngle, horzMaxAngle;
  int horzSamples = 1;
  // double horzResolution = 1.0;

  double pDiff = 0;
  int vertSamples = 1;
  // double vertResolution = 1.0;
  double vertMinAngle = 0;

  // Min range of a ray
  double minRange = 0;

  // Max range of a ray
  double maxRange = 1000;

  this->multiRayShapeDPtr->rayElem =
    this->multiRayShapeDPtr->sdf->GetElement("ray");
  this->multiRayShapeDPtr->scanElem =
    this->multiRayShapeDPtr->rayElem->GetElement("scan");
  this->multiRayShapeDPtr->horzElem =
    this->multiRayShapeDPtr->scanElem->GetElement("horizontal");
  this->multiRayShapeDPtr->rangeElem =
    this->multiRayShapeDPtr->rayElem->GetElement("range");

  if (this->multiRayShapeDPtr->scanElem->HasElement("vertical"))
  {
    this->multiRayShapeDPtr->vertElem =
      this->multiRayShapeDPtr->scanElem->GetElement("vertical");
    vertMinAngle = this->multiRayShapeDPtr->vertElem->Get<double>("min_angle");
    double vertMaxAngle = this->multiRayShapeDPtr->vertElem->Get<double>(
        "max_angle");
    vertSamples = this->multiRayShapeDPtr->vertElem->Get<unsigned int>(
        "samples");
    // vertResolution = this->multiRayShapeDPtr->vertElem->Get<double>(
    // "resolution");
    pDiff = vertMaxAngle - vertMinAngle;
  }

  horzMinAngle = this->multiRayShapeDPtr->horzElem->Get<double>("min_angle");
  horzMaxAngle = this->multiRayShapeDPtr->horzElem->Get<double>("max_angle");
  horzSamples = this->multiRayShapeDPtr->horzElem->Get<unsigned int>("samples");
  // horzResolution = this->multiRayShapeDPtr->horzElem->Get<double>(
  // "resolution");
  yDiff = horzMaxAngle - horzMinAngle;

  minRange = this->multiRayShapeDPtr->rangeElem->Get<double>("min");
  maxRange = this->multiRayShapeDPtr->rangeElem->Get<double>("max");

  this->multiRayShapeDPtr->offset =
    this->multiRayShapeDPtr->collisionParent->RelativePose();

  // Create an array of ray collisions
  for (unsigned int j = 0; j < (unsigned int)vertSamples; ++j)
  {
    for (unsigned int i = 0; i < (unsigned int)horzSamples; ++i)
    {
      yawAngle = (horzSamples == 1) ? 0 :
        i * yDiff / (horzSamples - 1) + horzMinAngle;

      pitchAngle = (vertSamples == 1)? 0 :
        j * pDiff / (vertSamples - 1) + vertMinAngle;

      // since we're rotating a unit x vector, a pitch rotation will now be
      // around the negative y axis
      ray.Euler(ignition::math::Vector3d(0.0, -pitchAngle, yawAngle));
      axis = this->multiRayShapeDPtr->offset.Rot() * ray *
        ignition::math::Vector3d(1.0, 0.0, 0.0);

      start = (axis * minRange) + this->multiRayShapeDPtr->offset.Pos();
      end = (axis * maxRange) + this->multiRayShapeDPtr->offset.Pos();

      this->AddRay(start, end);
    }
  }
}

//////////////////////////////////////////////////
void MultiRayShape::SetScale(const math::Vector3 &_scale)
{
  this->SetScale(_scale.Ign());
}

//////////////////////////////////////////////////
void MultiRayShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (this->multiRayShapeDPtr->scale == _scale)
    return;

  this->multiRayShapeDPtr->scale = _scale;

  for (unsigned int i = 0; i < this->multiRayShapeDPtr->rays.size(); ++i)
  {
    this->multiRayShapeDPtr->rays[i]->SetScale(this->multiRayShapeDPtr->scale);
  }
}

//////////////////////////////////////////////////
double MultiRayShape::GetRange(unsigned int _index)
{
  return this->Range(_index);
}

//////////////////////////////////////////////////
double MultiRayShape::Range(const unsigned int _index) const
{
  if (_index >= this->multiRayShapeDPtr->rays.size())
  {
    gzerr << "index[" << _index << "] out of range[0-"
      << this->multiRayShapeDPtr->rays.size() << "]\n";
    return IGN_DBL_INF;
  }

  // Add min range, because we measured from min range.
  return this->MinRange() + this->multiRayShapeDPtr->rays[_index]->Length();
}

//////////////////////////////////////////////////
double MultiRayShape::GetRetro(unsigned int _index)
{
  return this->Retro(_index);
}

//////////////////////////////////////////////////
double MultiRayShape::Retro(const unsigned int _index) const
{
  if (_index >= this->multiRayShapeDPtr->rays.size())
  {
    gzerr << "index[" << _index << "] out of range[0-"
      << this->multiRayShapeDPtr->rays.size() << "]\n";
    return IGN_DBL_INF;
  }

  return this->multiRayShapeDPtr->rays[_index]->Retro();
}

//////////////////////////////////////////////////
int MultiRayShape::GetFiducial(unsigned int _index)
{
  return this->Fiducial(_index);
}

//////////////////////////////////////////////////
int MultiRayShape::Fiducial(const unsigned int _index) const
{
  if (_index >= this->multiRayShapeDPtr->rays.size())
  {
    gzerr << "index[" << _index << "] out of range[0-"
      << this->multiRayShapeDPtr->rays.size() << "]\n";
    return IGN_INT32_INF;
  }

  return this->multiRayShapeDPtr->rays[_index]->Fiducial();
}

//////////////////////////////////////////////////
void MultiRayShape::Update()
{
  // The measurable range is (max-min)
  double fullRange = this->MaxRange() - this->MinRange();

  // Reset the ray lengths and mark the collisions as dirty (so they get
  // redrawn)
  unsigned int raySize = this->multiRayShapeDPtr->rays.size();
  for (unsigned int i = 0; i < raySize; ++i)
  {
    this->multiRayShapeDPtr->rays[i]->SetLength(fullRange);
    this->multiRayShapeDPtr->rays[i]->SetRetro(0.0);

    // Get the global points of the line
    this->multiRayShapeDPtr->rays[i]->Update();
  }

  // do actual collision checks
  this->UpdateRays();

  // for plugin
  this->multiRayShapeDPtr->newLaserScans();
}

//////////////////////////////////////////////////
bool MultiRayShape::SetRay(const unsigned int _rayIndex,
    const ignition::math::Vector3d &_start,
    const ignition::math::Vector3d &_end)
{
  if (_rayIndex < this->multiRayShapeDPtr->rays.size())
  {
    this->multiRayShapeDPtr->rays[_rayIndex]->SetPoints(_start, _end);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
void MultiRayShape::AddRay(const math::Vector3 &_start,
                           const math::Vector3 &_end)
{
  this->AddRay(_start.Ign(), _end.Ign());
}

//////////////////////////////////////////////////
void MultiRayShape::AddRay(const ignition::math::Vector3d &/*_start*/,
                           const ignition::math::Vector3d &/*_end*/)
{
  // msgs::Vector3d *pt = NULL;

  // FIXME: need to lock this when spawning models with ray.
  // This fails because RaySensor::laserShape->Update()
  // is called before rays could be constructed.
}

//////////////////////////////////////////////////
double MultiRayShape::GetMinRange() const
{
  return this->MinRange();
}

//////////////////////////////////////////////////
double MultiRayShape::MinRange() const
{
  return this->multiRayShapeDPtr->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double MultiRayShape::GetMaxRange() const
{
  return this->MaxRange();
}

//////////////////////////////////////////////////
double MultiRayShape::MaxRange() const
{
  return this->multiRayShapeDPtr->rangeElem->Get<double>("max");
}

//////////////////////////////////////////////////
double MultiRayShape::GetResRange() const
{
  return this->ResolutionRange();
}

//////////////////////////////////////////////////
double MultiRayShape::ResolutionRange() const
{
  return this->multiRayShapeDPtr->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int MultiRayShape::GetSampleCount() const
{
  return this->SampleCount();
}

//////////////////////////////////////////////////
int MultiRayShape::SampleCount() const
{
  return this->multiRayShapeDPtr->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
double MultiRayShape::GetScanResolution() const
{
  return this->ScanResolution();
}

//////////////////////////////////////////////////
double MultiRayShape::ScanResolution() const
{
  return this->multiRayShapeDPtr->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMinAngle() const
{
  return this->MinAngle();
}

//////////////////////////////////////////////////
ignition::math::Angle MultiRayShape::MinAngle() const
{
  return this->multiRayShapeDPtr->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMaxAngle() const
{
  return this->MaxAngle();
}

//////////////////////////////////////////////////
ignition::math::Angle MultiRayShape::MaxAngle() const
{
  return this->multiRayShapeDPtr->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
int MultiRayShape::GetVerticalSampleCount() const
{
  return this->VerticalSampleCount();
}

//////////////////////////////////////////////////
int MultiRayShape::VerticalSampleCount() const
{
  if (this->multiRayShapeDPtr->vertElem)
    return this->multiRayShapeDPtr->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
double MultiRayShape::GetVerticalScanResolution() const
{
  return this->VerticalScanResolution();
}

//////////////////////////////////////////////////
double MultiRayShape::VerticalScanResolution() const
{
  if (this->multiRayShapeDPtr->vertElem)
    return this->multiRayShapeDPtr->vertElem->Get<double>("resolution");
  else
    return 1;
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMinAngle() const
{
  return this->VerticalMinAngle();
}

//////////////////////////////////////////////////
ignition::math::Angle MultiRayShape::VerticalMinAngle() const
{
  if (this->multiRayShapeDPtr->vertElem)
    return this->multiRayShapeDPtr->vertElem->Get<double>("min_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMaxAngle() const
{
  return this->VerticalMaxAngle();
}

//////////////////////////////////////////////////
ignition::math::Angle MultiRayShape::VerticalMaxAngle() const
{
  if (this->multiRayShapeDPtr->vertElem)
    return this->multiRayShapeDPtr->vertElem->Get<double>("max_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
void MultiRayShape::FillMsg(msgs::Geometry &/*_msg*/)
{
}

//////////////////////////////////////////////////
void MultiRayShape::ProcessMsg(const msgs::Geometry &/*_msg*/)
{
}

//////////////////////////////////////////////////
double MultiRayShape::ComputeVolume() const
{
  return 0;
}

//////////////////////////////////////////////////
unsigned int MultiRayShape::RayCount() const
{
  return this->multiRayShapeDPtr->rays.size();
}

//////////////////////////////////////////////////
RayShapePtr MultiRayShape::Ray(const unsigned int _rayIndex) const
{
  if (_rayIndex < this->multiRayShapeDPtr->rays.size())
    return this->multiRayShapeDPtr->rays[_rayIndex];
  else
    return RayShapePtr();
}

//////////////////////////////////////////////////
event::ConnectionPtr MultiRayShape::ConnectNewLaserScans(
    std::function<void ()> _subscriber)
{
  return this->multiRayShapeDPtr->newLaserScans.Connect(_subscriber);
}

//////////////////////////////////////////////////
void MultiRayShape::DisconnectNewLaserScans(event::ConnectionPtr &_conn)
{
  this->multiRayShapeDPtr->newLaserScans.Disconnect(_conn);
}
