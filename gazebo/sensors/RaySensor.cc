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
#include "msgs/msgs.hh"

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
}

//////////////////////////////////////////////////
RaySensor::~RaySensor()
{
  delete this->mutex;

  this->laserCollision->Fini();
  this->laserCollision.reset();

  this->laserShape->Fini();
  this->laserShape.reset();
}

//////////////////////////////////////////////////
std::string RaySensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void RaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->scanPub = this->node->Advertise<msgs::LaserScan>(this->GetTopic());

  physics::PhysicsEnginePtr physicsEngine = this->world->GetPhysicsEngine();
  this->laserCollision = physicsEngine->CreateCollision("multiray",
      this->parentName);
  this->laserCollision->SetName("ray_sensor_collision");
  this->laserCollision->SetRelativePose(this->pose);

  this->laserShape = boost::dynamic_pointer_cast<physics::MultiRayShape>(
                     this->laserCollision->GetShape());
  this->laserShape->Load(this->sdf);
  this->laserShape->Init();

  this->parentEntity = this->world->GetEntity(this->parentName);
}

//////////////////////////////////////////////////
void RaySensor::Init()
{
  Sensor::Init();
  this->laserMsg.set_frame(this->parentName);
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
  return (this->GetAngleMax() - this->GetAngleMin()).Radian() /
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
  // todo: maybe should check against this->laserMsg.ranges_size()
  //       as users use this to loop through GetRange() calls
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
  boost::mutex::scoped_lock(*this->mutex);
  if (this->laserMsg.ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index >= this->laserMsg.ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->laserMsg.ranges(_index);
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
  // do the collision checks
  // this eventually call OnNewScans, so move mutex lock behind it in case
  // need to move mutex lock after this? or make the OnNewLaserScan connection
  // call somewhere else?
  this->laserShape->Update();
  this->lastMeasurementTime = this->world->GetSimTime();

  // moving this behind laserShape update
  this->mutex->lock();

  // Store the latest laser scans into laserMsg
  msgs::Set(this->laserMsg.mutable_world_pose(),
      this->parentEntity->GetWorldPose() + this->GetPose());
  this->laserMsg.set_angle_min(this->GetAngleMin().Radian());
  this->laserMsg.set_angle_max(this->GetAngleMax().Radian());
  this->laserMsg.set_angle_step(this->GetAngleResolution());

  this->laserMsg.set_range_min(this->GetRangeMin());
  this->laserMsg.set_range_max(this->GetRangeMax());

  this->laserMsg.clear_ranges();
  this->laserMsg.clear_intensities();

  // todo: add loop for vertical range count
  for (unsigned int j = 0; j < (unsigned int)this->GetVerticalRayCount(); j++)
  for (unsigned int i = 0; i < (unsigned int)this->GetRayCount(); i++)
  {
    this->laserMsg.add_ranges(
      this->laserShape->GetRange(j * this->GetRayCount() + i));
    this->laserMsg.add_intensities(
        this->laserShape->GetRetro(j * this->GetRayCount() + i));
  }
  this->mutex->unlock();

  if (this->scanPub)
    this->scanPub->Publish(this->laserMsg);
}
