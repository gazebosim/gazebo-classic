/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/MultiRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
MultiRayShape::MultiRayShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(MULTIRAY_SHAPE);
  this->SetName("multiray");
}

//////////////////////////////////////////////////
MultiRayShape::~MultiRayShape()
{
  this->rays.clear();
}

//////////////////////////////////////////////////
void MultiRayShape::Init()
{
  math::Vector3 start, end, axis;
  double yawAngle, pitchAngle;
  math::Quaternion ray;
  double yDiff;
  double horzMinAngle, horzMaxAngle;
  int horzSamples = 1;
  // double horzResolution = 1.0;

  double pDiff = 0;
  int vertSamples = 1;
  // double vertResolution = 1.0;
  double vertMinAngle = 0;

  double minRange, maxRange;

  this->rayElem = this->sdf->GetElement("ray");
  this->scanElem = this->rayElem->GetElement("scan");
  this->horzElem = this->scanElem->GetElement("horizontal");
  this->rangeElem = this->rayElem->GetElement("range");

  if (this->scanElem->HasElement("vertical"))
  {
    this->vertElem = this->scanElem->GetElement("vertical");
    vertMinAngle = this->vertElem->Get<double>("min_angle");
    double vertMaxAngle = this->vertElem->Get<double>("max_angle");
    vertSamples = this->vertElem->Get<unsigned int>("samples");
    // vertResolution = this->vertElem->Get<double>("resolution");
    pDiff = vertMaxAngle - vertMinAngle;
  }

  horzMinAngle = this->horzElem->Get<double>("min_angle");
  horzMaxAngle = this->horzElem->Get<double>("max_angle");
  horzSamples = this->horzElem->Get<unsigned int>("samples");
  // horzResolution = this->horzElem->Get<double>("resolution");
  yDiff = horzMaxAngle - horzMinAngle;

  minRange = this->rangeElem->Get<double>("min");
  maxRange = this->rangeElem->Get<double>("max");

  this->offset = this->collisionParent->GetRelativePose();

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
      ray.SetFromEuler(math::Vector3(0.0, -pitchAngle, yawAngle));
      axis = this->offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);

      start = (axis * minRange) + this->offset.pos;
      end = (axis * maxRange) + this->offset.pos;

      this->AddRay(start, end);
    }
  }
}

//////////////////////////////////////////////////
void MultiRayShape::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;

  for (unsigned int i = 0; i < this->rays.size(); ++i)
  {
    this->rays[i]->SetScale(this->scale);
  }
}

//////////////////////////////////////////////////
double MultiRayShape::GetRange(unsigned int _index)
{
  if (_index >= this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << _index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  // Add min range, because we measured from min range.
  return this->GetMinRange() + this->rays[_index]->GetLength();
}

//////////////////////////////////////////////////
double MultiRayShape::GetRetro(unsigned int _index)
{
  if (_index >= this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << _index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetRetro();
}

//////////////////////////////////////////////////
int MultiRayShape::GetFiducial(unsigned int _index)
{
  if (_index >= this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << _index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetFiducial();
}

//////////////////////////////////////////////////
void MultiRayShape::Update()
{
  // The measurable range is (max-min)
  double fullRange = this->GetMaxRange() - this->GetMinRange();

  // Reset the ray lengths and mark the collisions as dirty (so they get
  // redrawn)
  unsigned int ray_size = this->rays.size();
  for (unsigned int i = 0; i < ray_size; i++)
  {
    this->rays[i]->SetLength(fullRange);
    this->rays[i]->SetRetro(0.0);

    // Get the global points of the line
    this->rays[i]->Update();
  }

  // do actual collision checks
  this->UpdateRays();

  // for plugin
  this->newLaserScans();
}

//////////////////////////////////////////////////
void MultiRayShape::AddRay(const math::Vector3 &/*_start*/,
                           const math::Vector3 &/*_end*/)
{
  // msgs::Vector3d *pt = NULL;

  // FIXME: need to lock this when spawning models with ray.
  // This fails because RaySensor::laserShape->Update()
  // is called before rays could be constructed.
}


//////////////////////////////////////////////////
double MultiRayShape::GetMinRange() const
{
  return this->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double MultiRayShape::GetMaxRange() const
{
  return this->rangeElem->Get<double>("max");
}

//////////////////////////////////////////////////
double MultiRayShape::GetResRange() const
{
  return this->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int MultiRayShape::GetSampleCount() const
{
  return this->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
double MultiRayShape::GetScanResolution() const
{
  return this->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMinAngle() const
{
  return this->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMaxAngle() const
{
  return this->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
int MultiRayShape::GetVerticalSampleCount() const
{
  if (this->vertElem)
    return this->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
double MultiRayShape::GetVerticalScanResolution() const
{
  if (this->vertElem)
    return this->vertElem->Get<double>("resolution");
  else
    return 1;
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMinAngle() const
{
  if (this->vertElem)
    return this->vertElem->Get<double>("min_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMaxAngle() const
{
  if (this->vertElem)
    return this->vertElem->Get<double>("max_angle");
  else
    return math::Angle(0);
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
