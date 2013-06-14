/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/common/Exception.hh"
#include "msgs/msgs.hh"
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
  double vertMaxAngle = 0;

  double minRange, maxRange;

  this->rayElem = this->sdf->GetElement("ray");
  this->scanElem = this->rayElem->GetElement("scan");
  this->horzElem = this->scanElem->GetElement("horizontal");
  this->rangeElem = this->rayElem->GetElement("range");

  if (this->scanElem->HasElement("vertical"))
  {
    this->vertElem = this->scanElem->GetElement("vertical");
    vertMinAngle = this->vertElem->GetValueDouble("min_angle");
    vertMaxAngle = this->vertElem->GetValueDouble("max_angle");
    vertSamples = this->vertElem->GetValueUInt("samples");
    // vertResolution = this->vertElem->GetValueDouble("resolution");
    pDiff = vertMaxAngle - vertMinAngle;
  }

  horzMinAngle = this->horzElem->GetValueDouble("min_angle");
  horzMaxAngle = this->horzElem->GetValueDouble("max_angle");
  horzSamples = this->horzElem->GetValueUInt("samples");
  // horzResolution = this->horzElem->GetValueDouble("resolution");
  yDiff = horzMaxAngle - horzMinAngle;

  minRange = this->rangeElem->GetValueDouble("min");
  maxRange = this->rangeElem->GetValueDouble("max");

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

      ray.SetFromEuler(math::Vector3(0.0, pitchAngle, yawAngle));
      axis = this->offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);

      start = (axis * minRange) + this->offset.pos;
      end = (axis * maxRange) + this->offset.pos;

      this->AddRay(start, end);
    }
  }
}

//////////////////////////////////////////////////
double MultiRayShape::GetRange(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->rays.size()))
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
double MultiRayShape::GetRetro(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "index[" << _index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetRetro();
}

//////////////////////////////////////////////////
int MultiRayShape::GetFiducial(int _index)
{
  if (_index < 0 || _index >= static_cast<int>(this->rays.size()))
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
  return this->rangeElem->GetValueDouble("min");
}

//////////////////////////////////////////////////
double MultiRayShape::GetMaxRange() const
{
  return this->rangeElem->GetValueDouble("max");
}

//////////////////////////////////////////////////
double MultiRayShape::GetResRange() const
{
  return this->rangeElem->GetValueDouble("resolution");
}

//////////////////////////////////////////////////
int MultiRayShape::GetSampleCount() const
{
  return this->horzElem->GetValueUInt("samples");
}

//////////////////////////////////////////////////
double MultiRayShape::GetScanResolution() const
{
  return this->horzElem->GetValueDouble("resolution");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMinAngle() const
{
  return this->horzElem->GetValueDouble("min_angle");
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetMaxAngle() const
{
  return this->horzElem->GetValueDouble("max_angle");
}

//////////////////////////////////////////////////
int MultiRayShape::GetVerticalSampleCount() const
{
  if (this->vertElem)
    return this->vertElem->GetValueUInt("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
double MultiRayShape::GetVerticalScanResolution() const
{
  if (this->vertElem)
    return this->vertElem->GetValueDouble("resolution");
  else
    return 1;
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMinAngle() const
{
  if (this->vertElem)
    return this->vertElem->GetValueDouble("min_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
math::Angle MultiRayShape::GetVerticalMaxAngle() const
{
  if (this->vertElem)
    return this->vertElem->GetValueDouble("max_angle");
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
