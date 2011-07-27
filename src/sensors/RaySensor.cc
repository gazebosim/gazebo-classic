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
#include "physics/Physics.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Geom.hh"
#include "physics/World.hh"
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
  gzdbg << "New Ray Sensor\n";
  this->active = false;
  this->typeName = "ray";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RaySensor::~RaySensor()
{
  //if (this->laserGeom)
  //  delete this->laserGeom;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an SDF 
void RaySensor::Load( sdf::ElementPtr &_sdf )
{
  Sensor::Load(_sdf);
}
void RaySensor::Load( )
{
  std::string linkName = this->sdf->GetLinkName();
  //gzerr << "parent link name : " << linkName << "\n";

  std::string modelName = this->sdf->GetModelName();
  //gzerr << "parent model name : " << modelName << "\n";

  // get parent link by looking at real parent
  std::string linkFullyScopedName = "root::" + modelName + "::" + linkName;
  //gzerr << "scoped link name : " << linkFullyScopedName << "\n";

  std::string worldName = this->sdf->GetWorldName();
  //gzerr << "parent world name : " << worldName << "\n";


  worldName = "default"; // TODO: HACK!! for now there is only one default world
  this->world = gazebo::physics::get_world(worldName);
  this->model = this->world->GetModelByName(modelName);
  this->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->model->GetByName(linkFullyScopedName));

  if (this->link == NULL)
    gzthrow("Null link in the ray sensor");
  this->physicsEngine = this->world->GetPhysicsEngine();
  this->laserGeom = this->physicsEngine->CreateGeom(
      "multiray", this->link);

  this->laserGeom->SetName("Ray Sensor Geom");

  this->laserShape = boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(this->laserGeom->GetShape());

  this->laserShape->Load( this->sdf );
  this->laserShape->SetWorld(this->world);

  this->laserGeom->SetShape(this->laserShape);
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void RaySensor::InitChild()
{
  //gzerr << "Initializing RaySensor\n";
  this->laserShape->Init( );
  gazebo::math::Pose linkPose;
  linkPose = this->link->GetWorldPose();
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
void RaySensor::Update(bool force)
{
  this->physicsEngine->odeRaySensorMutex->lock();
  //if (this->active || (**this->alwaysActiveP))
    this->laserShape->Update();
  this->physicsEngine->odeRaySensorMutex->unlock();
}
