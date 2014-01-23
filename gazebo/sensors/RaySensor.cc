/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/RaySensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("ray", RaySensor)

//////////////////////////////////////////////////
RaySensor::RaySensor()
    : Sensor(sensors::RAY)
{
}

//////////////////////////////////////////////////
RaySensor::~RaySensor()
{
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
  this->scanPub = this->node->Advertise<msgs::LaserScanStamped>(
      this->GetTopic(), 50);

  GZ_ASSERT(this->world != NULL,
      "RaySensor did not get a valid World pointer");

  physics::PhysicsEnginePtr physicsEngine = this->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  this->laserCollision = physicsEngine->CreateCollision("multiray",
      this->parentName);

  GZ_ASSERT(this->laserCollision != NULL,
      "Unable to create a multiray collision using the physics engine.");

  this->laserCollision->SetName("ray_sensor_collision");
  this->laserCollision->SetRelativePose(this->pose);
  this->laserCollision->SetInitialRelativePose(this->pose);

  this->laserShape = boost::dynamic_pointer_cast<physics::MultiRayShape>(
                     this->laserCollision->GetShape());

  GZ_ASSERT(this->laserShape != NULL,
      "Unable to get the laser shape from the multi-ray collision.");

  this->laserShape->Load(this->sdf);
  this->laserShape->Init();

  // Handle noise model settings.
  this->noiseActive = false;
  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  if (rayElem->HasElement("noise"))
  {
    sdf::ElementPtr noiseElem = rayElem->GetElement("noise");
    std::string type = noiseElem->Get<std::string>("type");
    if (type == "gaussian")
    {
      this->noiseType = GAUSSIAN;
      this->noiseMean = noiseElem->Get<double>("mean");
      this->noiseStdDev = noiseElem->Get<double>("stddev");
      this->noiseActive = true;
      gzlog << "applying Gaussian noise model with mean " << this->noiseMean <<
        " and stddev " << this->noiseStdDev << std::endl;
    }
    else
      gzwarn << "ignoring unknown noise model type \"" << type << "\"" <<
        std::endl;
  }

  this->parentEntity = this->world->GetEntity(this->parentName);

  GZ_ASSERT(this->parentEntity != NULL,
      "Unable to get the parent entity.");
}

//////////////////////////////////////////////////
void RaySensor::Init()
{
  Sensor::Init();
  this->laserMsg.mutable_scan()->set_frame(this->parentName);
}

//////////////////////////////////////////////////
void RaySensor::Fini()
{
  Sensor::Fini();

  this->scanPub.reset();

  if (this->laserCollision)
  {
    this->laserCollision->Fini();
    this->laserCollision.reset();
  }

  if (this->laserShape)
  {
    this->laserShape->Fini();
    this->laserShape.reset();
  }
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetAngleMin() const
{
  if (this->laserShape)
    return this->laserShape->GetMinAngle();
  else
    return -1;
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetAngleMax() const
{
  if (this->laserShape)
    return this->laserShape->GetMaxAngle();
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMin() const
{
  if (this->laserShape)
    return this->laserShape->GetMinRange();
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMax() const
{
  if (this->laserShape)
    return this->laserShape->GetMaxRange();
  else
    return -1;
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
  if (this->laserShape)
    return this->laserShape->GetResRange();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetRayCount() const
{
  if (this->laserShape)
    return this->laserShape->GetSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetRangeCount() const
{
  // TODO: maybe should check against this->laserMsg.ranges_size()
  //       as users use this to loop through GetRange() calls
  if (this->laserShape)
    return this->laserShape->GetSampleCount() *
      this->laserShape->GetScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRayCount() const
{
  if (this->laserShape)
    return this->laserShape->GetVerticalSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRangeCount() const
{
  if (this->laserShape)
    return this->laserShape->GetVerticalSampleCount() *
      this->laserShape->GetVerticalScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetVerticalAngleMin() const
{
  if (this->laserShape)
    return this->laserShape->GetVerticalMinAngle();
  else
    return -1;
}

//////////////////////////////////////////////////
math::Angle RaySensor::GetVerticalAngleMax() const
{
  if (this->laserShape)
    return this->laserShape->GetVerticalMaxAngle();
  else
    return -1;
}

//////////////////////////////////////////////////
void RaySensor::GetRanges(std::vector<double> &_ranges)
{
  boost::mutex::scoped_lock lock(this->mutex);

  _ranges.resize(this->laserMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->laserMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->laserMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double RaySensor::GetRange(int _index)
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (this->laserMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index >= this->laserMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->laserMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double RaySensor::GetRetro(int index)
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (this->laserShape)
    return this->laserShape->GetRetro(index);
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetFiducial(int index)
{
  boost::mutex::scoped_lock lock(this->mutex);
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
  boost::mutex::scoped_lock lock(this->mutex);

  msgs::Set(this->laserMsg.mutable_time(), this->lastMeasurementTime);

  msgs::LaserScan *scan = this->laserMsg.mutable_scan();

  // Store the latest laser scans into laserMsg
  msgs::Set(scan->mutable_world_pose(),
            this->pose + this->parentEntity->GetWorldPose());
  scan->set_angle_min(this->GetAngleMin().Radian());
  scan->set_angle_max(this->GetAngleMax().Radian());
  scan->set_angle_step(this->GetAngleResolution());

  scan->set_range_min(this->GetRangeMin());
  scan->set_range_max(this->GetRangeMax());

  scan->clear_ranges();
  scan->clear_intensities();

  // todo: add loop for vertical range count
  for (unsigned int j = 0; j < (unsigned int)this->GetVerticalRayCount(); j++)
  for (unsigned int i = 0; i < (unsigned int)this->GetRayCount(); i++)
  {
    double range = this->laserShape->GetRange(j * this->GetRayCount() + i);
    if (this->noiseActive)
    {
      switch (this->noiseType)
      {
        case GAUSSIAN:
          // Add independent (uncorrelated) Gaussian noise to each beam.
          range += math::Rand::GetDblNormal(this->noiseMean, this->noiseStdDev);
          // No real laser would return a range outside its stated limits.
          range = math::clamp(range, this->GetRangeMin(), this->GetRangeMax());
          break;
        default:
          GZ_ASSERT(false, "Invalid noise model type");
      }
    }
    scan->add_ranges(range);
    scan->add_intensities(
        this->laserShape->GetRetro(j * this->GetRayCount() + i));
  }

  if (this->scanPub && this->scanPub->HasConnections())
    this->scanPub->Publish(this->laserMsg);
}

//////////////////////////////////////////////////
bool RaySensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->scanPub && this->scanPub->HasConnections());
}
