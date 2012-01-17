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
*/

#include "physics/World.hh"
#include "physics/MultiRayShape.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Physics.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"
#include "common/Exception.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"
#include "msgs/msgs.h"

#include "math/Vector3.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/RaySensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("ray", RaySensor)

//////////////////////////////////////////////////
RaySensor::RaySensor()
    : Sensor()
{
  this->active = false;
  this->node = transport::NodePtr(new transport::Node());
}

//////////////////////////////////////////////////
RaySensor::~RaySensor()
{
  this->laserCollision.reset();
  this->link.reset();
  this->laserShape.reset();
}

//////////////////////////////////////////////////
void RaySensor::Load(sdf::ElementPtr &_sdf)
{
  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////
void RaySensor::Load()
{
  Sensor::Load();

  std::string linkName = this->sdf->GetLinkName();
  std::string modelName = this->sdf->GetModelName();
  std::string worldName = this->sdf->GetWorldName();
  std::string linkFullyScopedName = worldName + "::" + modelName + "::" +
                                    linkName;

  if (this->sdf->GetElement("topic"))
  {
    this->node->Init(worldName);
    this->scanPub = this->node->Advertise<msgs::LaserScan>(
        this->sdf->GetElement("topic")->GetValueString());
  }

  physics::WorldPtr world = physics::get_world(worldName);
  this->link = world->GetModelByName(modelName)->GetChildLink(linkName);

  if (this->link == NULL)
    gzthrow("Null link in the ray sensor");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
  this->laserCollision = physicsEngine->CreateCollision("multiray", this->link);
  this->laserCollision->SetName("ray_sensor_collision");
  this->laserCollision->SetRelativePose(this->pose);

  this->laserShape = boost::dynamic_pointer_cast<physics::MultiRayShape>(
      this->laserCollision->GetShape());

  this->laserShape->Load(this->sdf);

  this->laserShape->Init();
}

//////////////////////////////////////////////////
void RaySensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void RaySensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetAngleMin() const
{
  return this->laserShape->GetMinAngle();
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetAngleMax() const
{
  return this->laserShape->GetMaxAngle();
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMin() const
{
  return this->laserShape->GetMinRange();
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMax() const
{
  return this->laserShape->GetMaxRange();
}

//////////////////////////////////////////////////
double RaySensor::GetAngleResolution() const
{
  return (this->GetAngleMax() - this->GetAngleMin()).GetAsRadian() /
    (this->GetRangeCount()-1);
}

//////////////////////////////////////////////////
double RaySensor::GetRangeResolution() const
{
  return this->laserShape->GetResRange();
}

//////////////////////////////////////////////////
int RaySensor::GetRayCount() const
{
  return this->laserShape->GetSampleCount();
}

//////////////////////////////////////////////////
int RaySensor::GetRangeCount() const
{
  return this->laserShape->GetSampleCount() *
         this->laserShape->GetScanResolution();
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRayCount() const
{
  return this->laserShape->GetVerticalSampleCount();
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRangeCount() const
{
  return this->laserShape->GetVerticalSampleCount() *
         this->laserShape->GetVerticalScanResolution();
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetVerticalAngleMin() const
{
  return this->laserShape->GetVerticalMinAngle();
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetVerticalAngleMax() const
{
  return this->laserShape->GetVerticalMaxAngle();
}

//////////////////////////////////////////////////
double RaySensor::GetRange(int index)
{
  return this->laserShape->GetRange(index);
}

//////////////////////////////////////////////////
double RaySensor::GetRetro(int index)
{
  return this->laserShape->GetRetro(index);
}

//////////////////////////////////////////////////
int RaySensor::GetFiducial(int index)
{
  return this->laserShape->GetFiducial(index);
}

//////////////////////////////////////////////////
void RaySensor::UpdateImpl(bool /*_force*/)
{
  this->laserShape->Update();
  this->lastUpdateTime = this->link->GetWorld()->GetSimTime();

  if (this->scanPub)
  {
    msgs::LaserScan msg;

    msg.set_frame(this->link->GetScopedName());
    msgs::Set(msg.mutable_offset(), this->GetPose());
    msg.set_angle_min(this->GetAngleMin().GetAsRadian());
    msg.set_angle_max(this->GetAngleMax().GetAsRadian());
    msg.set_angle_step(this->GetAngleResolution());

    msg.set_range_min(this->GetRangeMin());
    msg.set_range_max(this->GetRangeMax());

    for (unsigned int i = 0; i < (unsigned int)this->GetRangeCount(); i++)
    {
      msg.add_ranges(this->laserShape->GetRange(i));
      msg.add_intensities(0);
    }

    this->scanPub->Publish(msg);
  }
}

