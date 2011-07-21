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
/* Desc: Ray proximity sensor
 * Author: Carle Cote
 * Date: 23 february 2004
 * SVN: $Id$
*/

#include "physics/World.hh"
#include "physics/MultiRayShape.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Geom.hh"
#include "common/Exception.hh"

#include "math/Vector3.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/RaySensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("ray", RaySensor)

//////////////////////////////////////////////////////////////////////////////
// Constructor
RaySensor::RaySensor()
    : Sensor()
{
  this->active = false;
  this->typeName = "ray";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RaySensor::~RaySensor()
{
  if (this->laserGeom)
    delete this->laserGeom;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an SDF 
void RaySensor::LoadChild( sdf::ElementPtr &_sdf )
{
  gzerr << "LOADING RaySensor\n";

/*
  if (this->body == NULL)
    gzthrow("Null body in the ray sensor");

  this->laserGeom = this->GetWorld()->GetPhysicsEngine()->CreateGeom(
      "multiray", this->body);
*/
  this->laserGeom->SetName("Ray Sensor Geom");

/* use boost cast
  this->laserShape = (gazebo::physics::MultiRayShape*)(this->laserGeom->GetShape());
*/

  this->laserShape->Load( _sdf );
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void RaySensor::InitChild()
{
  gazebo::math::Pose bodyPose;
/*
  bodyPose = this->body->GetWorldPose();
*/
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void RaySensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum angle
gazebo::math::Angle RaySensor::GetMinAngle() const
{
  return this->laserShape->GetMinAngle();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the maximum angle
gazebo::math::Angle RaySensor::GetMaxAngle() const
{
  return this->laserShape->GetMaxAngle();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum range
double RaySensor::GetMinRange() const
{
  return this->laserShape->GetMinRange();
}

//////////////////////////////////////////////////////////////////////////////
///  Get the maximum range
double RaySensor::GetMaxRange() const
{
  return this->laserShape->GetMaxRange();
}

//////////////////////////////////////////////////////////////////////////////
///  Get the range resolution
double RaySensor::GetResRange() const
{
  return this->laserShape->GetResRange();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the ray count
int RaySensor::GetRayCount() const
{
  return this->GetRayCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the range count
int RaySensor::GetRangeCount() const
{
  return this->GetRangeCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int RaySensor::GetVerticalRayCount() const
{
  return this->GetVerticalRayCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int RaySensor::GetVerticalRangeCount() const
{
  return this->GetVerticalRangeCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical min angle
gazebo::math::Angle RaySensor::GetVerticalMinAngle() const
{
  return this->laserShape->GetVerticalMinAngle();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical max angle
gazebo::math::Angle RaySensor::GetVerticalMaxAngle() const
{
  return this->laserShape->GetVerticalMaxAngle();
}

//////////////////////////////////////////////////////////////////////////////
// Get detected range for a ray
double RaySensor::GetRange(int index)
{
  return this->laserShape->GetRange(index);
}

//////////////////////////////////////////////////////////////////////////////
// Get detected retro (intensity) value for a ray.
double RaySensor::GetRetro(int index)
{
  return this->laserShape->GetRetro(index);
}

//////////////////////////////////////////////////////////////////////////////
// Get detected fiducial value for a ray.
int RaySensor::GetFiducial(int index)
{
  return this->laserShape->GetFiducial(index);
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void RaySensor::UpdateChild()
{
/*
  if (this->active || (**this->alwaysActiveP))
    this->laserShape->Update();
*/
}
