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
  this->mutex = new boost::mutex();
  this->active = false;
  this->node = transport::NodePtr(new transport::Node());
}

//////////////////////////////////////////////////
RaySensor::~RaySensor()
{
  delete this->mutex;
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

  if (this->sdf->GetElement("topic"))
  {
    this->node->Init(worldName);
    this->scanPub = this->node->Advertise<msgs::LaserScan>(
        this->sdf->GetElement("topic")->GetValueString());
  }

  physics::WorldPtr worldPtr = physics::get_world(worldName);
  this->link = worldPtr->GetModelByName(modelName)->GetChildLink(linkName);

  if (this->link == NULL)
    gzthrow("Null link in the ray sensor");

  physics::PhysicsEnginePtr physicsEngine = worldPtr->GetPhysicsEngine();
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
  this->laserMsg.set_frame(this->link->GetScopedName());
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
void RaySensor::GetRanges(std::vector<double> &_ranges)
{
  boost::mutex::scoped_lock(*this->mutex);
  _ranges.resize(this->laserMsg.ranges_size());
  memcpy(&_ranges[0], this->laserMsg.ranges().data(),
         sizeof(_ranges[0]) * this->laserMsg.ranges_size());
}

//////////////////////////////////////////////////
double RaySensor::GetRange(int _index)
{
  if (_index < 0 || _index > this->laserMsg.ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  boost::mutex::scoped_lock(*this->mutex);
  return this->laserMsg.ranges(_index);
  //return this->laserShape->GetRange(index);
}

//////////////////////////////////////////////////
double RaySensor::GetRetro(int index)
{
  boost::mutex::scoped_lock(*this->mutex);
  return this->laserShape->GetRetro(index);
}

//////////////////////////////////////////////////
int RaySensor::GetFiducial(int index)
{
  boost::mutex::scoped_lock(*this->mutex);
  return this->laserShape->GetFiducial(index);
}

//////////////////////////////////////////////////
void RaySensor::UpdateImpl(bool /*_force*/)
{
  this->mutex->lock();
  this->laserShape->Update();
  this->lastUpdateTime = this->link->GetWorld()->GetSimTime();

  // Store the latest laser scan.
  msgs::Set(this->laserMsg.mutable_offset(), this->GetPose());
  this->laserMsg.set_angle_min(this->GetAngleMin().GetAsRadian());
  this->laserMsg.set_angle_max(this->GetAngleMax().GetAsRadian());
  this->laserMsg.set_angle_step(this->GetAngleResolution());

  this->laserMsg.set_range_min(this->GetRangeMin());
  this->laserMsg.set_range_max(this->GetRangeMax());
  this->laserMsg.clear_ranges();
  this->laserMsg.clear_intensities();

  for (unsigned int i = 0; i < (unsigned int)this->GetRangeCount(); i++)
  {
    this->laserMsg.add_ranges(this->laserShape->GetRange(i));
    this->laserMsg.add_intensities(0);
  }
  this->mutex->unlock();

  if (this->scanPub)
    this->scanPub->Publish(this->laserMsg);
}
