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
#include "physics/Collision.hh"
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
  this->active = false;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RaySensor::~RaySensor()
{
  //if (this->laserCollision)
  //  delete this->laserCollision;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an SDF 
void RaySensor::Load( sdf::ElementPtr &_sdf )
{
  Sensor::Load(_sdf);
}
void RaySensor::Load( )
{
  Sensor::Load();

  std::string linkName = this->sdf->GetLinkName();
  //gzerr << "parent link name : " << linkName << "\n";

  std::string modelName = this->sdf->GetModelName();
  //gzerr << "parent model name : " << modelName << "\n";

  std::string worldName = this->sdf->GetWorldName();
  //gzerr << "parent world name : " << worldName << "\n";

  // get parent link by looking at real parent
  std::string linkFullyScopedName = worldName + "::" + modelName + "::" + linkName;
  //gzerr << "scoped link name : " << linkFullyScopedName << "\n";


  this->world = gazebo::physics::get_world(worldName);
  this->model = this->world->GetModelByName(modelName);
  gazebo::physics::BasePtr tmp = this->model->GetByName(linkFullyScopedName);
  printf("ok\n");
  this->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->model->GetByName(linkFullyScopedName));

  if (this->link == NULL)
    gzthrow("Null link in the ray sensor");
  this->physicsEngine = this->world->GetPhysicsEngine();
  this->laserCollision = this->physicsEngine->CreateCollision(
      "multiray", this->link);

  this->laserCollision->SetName("Ray Sensor Collision");

  this->laserShape = boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(this->laserCollision->GetShape());

  this->laserShape->Load( this->sdf );
  this->laserShape->SetWorld(this->world);

  this->laserCollision->SetShape(this->laserShape);

  this->laserShape->Init( );
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void RaySensor::Init()
{
  //gzerr << "Initializing RaySensor\n";
  //this->laserShape->Init( );
  //gazebo::math::Pose linkPose;
  //linkPose = this->link->GetWorldPose();

  Sensor::Init();
}

//////////////////////////////////////////////////////////////////////////////
// Fini the ray
void RaySensor::Fini()
{
  Sensor::Fini();
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
  return this->laserShape->GetSampleCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the range count
int RaySensor::GetRangeCount() const
{
  return this->laserShape->GetSampleCount()*this->laserShape->GetScanResolution();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int RaySensor::GetVerticalRayCount() const
{
  return this->laserShape->GetVerticalSampleCount();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int RaySensor::GetVerticalRangeCount() const
{
  return this->laserShape->GetVerticalSampleCount()*this->laserShape->GetVerticalScanResolution();
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
void RaySensor::UpdateImpl(bool /*_force*/)
{
  this->physicsEngine->odeRaySensorMutex->lock();
  //if (this->active || (**this->alwaysActiveP))
  // FIXME:  There is a race condition that causes below to be called before laserShape is fully
  //         instantiated when spawning a model
  this->laserShape->Update();
  this->physicsEngine->odeRaySensorMutex->unlock();
  this->lastUpdateTime = this->world->GetSimTime();
}
