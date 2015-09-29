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
/* Desc: Ray proximity sensor
 * Author: Carle Cote
 * Date: 23 february 2004
*/
#include <boost/algorithm/string.hpp>

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
#include "gazebo/sensors/Noise.hh"

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
  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  if (rayElem->HasElement("noise"))
  {
    this->noises.push_back(
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->GetType()));
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
double RaySensor::GetVerticalAngleResolution() const
{
  return (this->GetVerticalAngleMax() - this->GetVerticalAngleMin()).Radian() /
    (this->GetVerticalRangeCount()-1);
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
double RaySensor::GetRange(unsigned int _index)
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (this->laserMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >= this->laserMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->laserMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double RaySensor::GetRetro(unsigned int _index)
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (this->laserMsg.scan().intensities_size() == 0)
  {
    gzwarn << "Intensities not constructed yet (zero size)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >= this->laserMsg.scan().intensities_size())
  {
    gzerr << "Invalid intensity index[" << _index << "]\n";
    return 0.0;
  }

  return this->laserMsg.scan().intensities(_index);
}

//////////////////////////////////////////////////
int RaySensor::GetFiducial(unsigned int _index)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Convert range index to ray index.
  // Find vertical/horizontal range indices (vIdx, hIdx) and mulitply
  // by the ratio of ray count to range count to get the vertical/horizontal
  // ray indices, which are then used to compute the final index into ray array.
  int vIdx = _index / this->GetRangeCount();
  vIdx = vIdx * this->GetVerticalRayCount() / this->GetVerticalRangeCount();
  int hIdx = _index % this->GetRangeCount();
  hIdx = hIdx * this->GetRayCount() / this->GetRangeCount();
  int idx = vIdx * this->GetRayCount()  + hIdx;

  if (idx >=  this->GetRayCount() * this->GetVerticalRayCount())
  {
    gzerr << "Invalid fiducial index[" << _index << "]\n";
    return 0.0;
  }
  return this->laserShape->GetFiducial(idx);
}

//////////////////////////////////////////////////
bool RaySensor::UpdateImpl(bool /*_force*/)
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
  scan->set_count(this->GetRangeCount());

  scan->set_vertical_angle_min(this->GetVerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->GetVerticalAngleMax().Radian());
  scan->set_vertical_angle_step(this->GetVerticalAngleResolution());
  scan->set_vertical_count(this->GetVerticalRangeCount());

  scan->set_range_min(this->GetRangeMin());
  scan->set_range_max(this->GetRangeMax());

  scan->clear_ranges();
  scan->clear_intensities();

  unsigned int rayCount = this->GetRayCount();
  unsigned int rangeCount = this->GetRangeCount();
  unsigned int verticalRayCount = this->GetVerticalRayCount();
  unsigned int verticalRangeCount = this->GetVerticalRangeCount();

  // Interpolation: for every point in range count, compute interpolated value
  // using four bounding ray samples.
  // (vja, hja)   (vja, hjb)
  //       x---------x
  //       |         |
  //       |    o    |
  //       |         |
  //       x---------x
  // (vjb, hja)   (vjb, hjb)
  // where o: is the range to be interpolated
  //       x: ray sample
  //       vja: is the previous index of ray in vertical direction
  //       vjb: is the next index of ray in vertical direction
  //       hja: is the previous index of ray in horizontal direction
  //       hjb: is the next index of ray in horizontal direction
  unsigned int hja, hjb;
  unsigned int vja = 0, vjb = 0;
  // percentage of interpolation between rays
  double vb = 0, hb;
  // indices of ray samples
  int j1, j2, j3, j4;
  // range values of ray samples
  double r1, r2, r3, r4;

  // Check for the common case of vertical and horizontal resolution being 1,
  // which means that ray count == range count and we can do simple lookup
  // of ranges and intensity data, skipping interpolation.  We could do this
  // check independently for vertical and horizontal, but that's more
  // complexity for an unlikely use case.
  bool interp =
    ((rayCount != rangeCount) || (verticalRayCount != verticalRangeCount));

  // interpolate in vertical direction
  for (unsigned int j = 0; j < verticalRangeCount; ++j)
  {
    if (interp)
    {
      vb = (verticalRangeCount == 1) ? 0 :
          static_cast<double>(j * (verticalRayCount - 1))
          / (verticalRangeCount - 1);
      vja = static_cast<int>(floor(vb));
      vjb = std::min(vja + 1, verticalRayCount - 1);
      vb = vb - floor(vb);

      GZ_ASSERT(vja < verticalRayCount,
          "Invalid vertical ray index used for interpolation");
      GZ_ASSERT(vjb < verticalRayCount,
          "Invalid vertical ray index used for interpolation");
    }
    // interpolate in horizontal direction
    for (unsigned int i = 0; i < rangeCount; ++i)
    {
      double range, intensity;
      if (interp)
      {
        hb = (rangeCount == 1)? 0 : static_cast<double>(i * (rayCount - 1))
            / (rangeCount - 1);
        hja = static_cast<int>(floor(hb));
        hjb = std::min(hja + 1, rayCount - 1);
        hb = hb - floor(hb);

        GZ_ASSERT(hja < rayCount,
            "Invalid horizontal ray index used for interpolation");
        GZ_ASSERT(hjb < rayCount,
            "Invalid horizontal ray index used for interpolation");

        // indices of 4 corners
        j1 = hja + vja * rayCount;
        j2 = hjb + vja * rayCount;
        j3 = hja + vjb * rayCount;
        j4 = hjb + vjb * rayCount;

        // range readings of 4 corners
        r1 = this->GetLaserShape()->GetRange(j1);
        r2 = this->GetLaserShape()->GetRange(j2);
        r3 = this->GetLaserShape()->GetRange(j3);
        r4 = this->GetLaserShape()->GetRange(j4);
        range = (1-vb)*((1 - hb) * r1 + hb * r2)
            + vb *((1 - hb) * r3 + hb * r4);

        // intensity is averaged
        intensity = 0.25 * (this->GetLaserShape()->GetRetro(j1)
            + this->GetLaserShape()->GetRetro(j2)
            + this->GetLaserShape()->GetRetro(j3)
            + this->GetLaserShape()->GetRetro(j4));
      }
      else
      {
        range = this->laserShape->GetRange(j * this->GetRayCount() + i);
        intensity = this->laserShape->GetRetro(j * this->GetRayCount() + i);
      }

      if (!this->noises.empty())
      {
        // currently supports only one noise model per laser sensor
        range = this->noises[0]->Apply(range);
        range = math::clamp(range, this->GetRangeMin(), this->GetRangeMax());
      }

      scan->add_ranges(range);
      scan->add_intensities(intensity);
    }
  }

  if (this->scanPub && this->scanPub->HasConnections())
    this->scanPub->Publish(this->laserMsg);

  return true;
}

//////////////////////////////////////////////////
bool RaySensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->scanPub && this->scanPub->HasConnections());
}
