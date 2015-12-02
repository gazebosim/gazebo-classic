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
#include "gazebo/sensors/RaySensorPrivate.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/Noise.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("ray", RaySensor)

//////////////////////////////////////////////////
RaySensor::RaySensor()
: Sensor(*new RaySensorPrivate, sensors::RAY),
  dataPtr(std::static_pointer_cast<RaySensorPrivate>(this->dPtr))
{
}

//////////////////////////////////////////////////
RaySensor::~RaySensor()
{
}

//////////////////////////////////////////////////
std::string RaySensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void RaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->dataPtr->scanPub =
    this->dataPtr->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

  GZ_ASSERT(this->dataPtr->world != NULL,
      "RaySensor did not get a valid World pointer");

  physics::PhysicsEnginePtr physicsEngine =
    this->dataPtr->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  this->dataPtr->laserCollision = physicsEngine->CreateCollision("multiray",
      this->ParentName());

  GZ_ASSERT(this->dataPtr->laserCollision != NULL,
      "Unable to create a multiray collision using the physics engine.");

  this->dataPtr->laserCollision->SetName("ray_sensor_collision");
  this->dataPtr->laserCollision->SetRelativePose(this->dataPtr->pose);
  this->dataPtr->laserCollision->SetInitialRelativePose(this->dataPtr->pose);

  this->dataPtr->laserShape =
    boost::dynamic_pointer_cast<physics::MultiRayShape>(
        this->dataPtr->laserCollision->GetShape());

  GZ_ASSERT(this->dataPtr->laserShape != NULL,
      "Unable to get the laser shape from the multi-ray collision.");

  this->dataPtr->laserShape->Load(this->dataPtr->sdf);
  this->dataPtr->laserShape->Init();

  // Handle noise model settings.
  sdf::ElementPtr rayElem = this->dataPtr->sdf->GetElement("ray");
  if (rayElem->HasElement("noise"))
  {
    this->dataPtr->noises[RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->dataPtr->world->GetEntity(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != NULL,
      "Unable to get the parent entity.");
}

//////////////////////////////////////////////////
void RaySensor::Init()
{
  Sensor::Init();
  this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

//////////////////////////////////////////////////
void RaySensor::Fini()
{
  Sensor::Fini();

  this->dataPtr->scanPub.reset();

  if (this->dataPtr->laserCollision)
  {
    this->dataPtr->laserCollision->Fini();
    this->dataPtr->laserCollision.reset();
  }

  if (this->dataPtr->laserShape)
  {
    this->dataPtr->laserShape->Fini();
    this->dataPtr->laserShape.reset();
  }
}

//////////////////////////////////////////////////
ignition::math::Angle RaySensor::AngleMin() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMinAngle().Ign();
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle RaySensor::AngleMax() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetMaxAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMin() const
{
  return this->RangeMin();
}

//////////////////////////////////////////////////
double RaySensor::RangeMin() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMinRange();
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetRangeMax() const
{
  return this->RangeMax();
}

//////////////////////////////////////////////////
double RaySensor::RangeMax() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMaxRange();
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetAngleResolution() const
{
  return this->AngleResolution();
}

//////////////////////////////////////////////////
double RaySensor::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double RaySensor::GetRangeResolution() const
{
  return this->RangeResolution();
}

//////////////////////////////////////////////////
double RaySensor::RangeResolution() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetResRange();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetRayCount() const
{
  return this->RayCount();
}

//////////////////////////////////////////////////
int RaySensor::RayCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetRangeCount() const
{
  return this->RangeCount();
}

//////////////////////////////////////////////////
int RaySensor::RangeCount() const
{
  // TODO: maybe should check against this->dataPtr->laserMsg.ranges_size()
  //       as users use this to loop through GetRange() calls
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetSampleCount() *
      this->dataPtr->laserShape->GetScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRayCount() const
{
  return this->VerticalRayCount();
}

//////////////////////////////////////////////////
int RaySensor::VerticalRayCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetVerticalSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int RaySensor::GetVerticalRangeCount() const
{
  return this->VerticalRangeCount();
}

//////////////////////////////////////////////////
int RaySensor::VerticalRangeCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetVerticalSampleCount() *
      this->dataPtr->laserShape->GetVerticalScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle RaySensor::VerticalAngleMin() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetVerticalMinAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle RaySensor::VerticalAngleMax() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetVerticalMaxAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
double RaySensor::GetVerticalAngleResolution() const
{
  return this->VerticalAngleResolution();
}

//////////////////////////////////////////////////
double RaySensor::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void RaySensor::GetRanges(std::vector<double> &_ranges)
{
  this->Ranges(_ranges);
}

//////////////////////////////////////////////////
void RaySensor::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  _ranges.resize(this->dataPtr->laserMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->dataPtr->laserMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->laserMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double RaySensor::GetRange(unsigned int _index)
{
  return this->Range(_index);
}

//////////////////////////////////////////////////
double RaySensor::Range(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->laserMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >= this->dataPtr->laserMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double RaySensor::GetRetro(unsigned int _index)
{
  return this->Retro(_index);
}

//////////////////////////////////////////////////
double RaySensor::Retro(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->laserMsg.scan().intensities_size() == 0)
  {
    gzwarn << "Intensities not constructed yet (zero size)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >=
      this->dataPtr->laserMsg.scan().intensities_size())
  {
    gzerr << "Invalid intensity index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.scan().intensities(_index);
}

//////////////////////////////////////////////////
int RaySensor::GetFiducial(unsigned int _index)
{
  return this->Fiducial(_index);
}

//////////////////////////////////////////////////
int RaySensor::Fiducial(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Convert range index to ray index.
  // Find vertical/horizontal range indices (vIdx, hIdx) and mulitply
  // by the ratio of ray count to range count to get the vertical/horizontal
  // ray indices, which are then used to compute the final index into ray array.
  int vIdx = _index / this->RangeCount();
  vIdx = vIdx * this->VerticalRayCount() / this->VerticalRangeCount();
  int hIdx = _index % this->RangeCount();
  hIdx = hIdx * this->RayCount() / this->RangeCount();
  int idx = vIdx * this->RayCount()  + hIdx;

  if (idx >= this->RayCount() * this->VerticalRayCount())
  {
    gzerr << "Invalid fiducial index[" << _index << "]\n";
    return 0.0;
  }
  return this->dataPtr->laserShape->GetFiducial(idx);
}

//////////////////////////////////////////////////
bool RaySensor::UpdateImpl(const bool /*_force*/)
{
  // do the collision checks
  // this eventually call OnNewScans, so move mutex lock behind it in case
  // need to move mutex lock after this? or make the OnNewLaserScan connection
  // call somewhere else?
  this->dataPtr->laserShape->Update();
  this->dataPtr->lastMeasurementTime = this->dataPtr->world->GetSimTime();

  // moving this behind laserShape update
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  msgs::Set(this->dataPtr->laserMsg.mutable_time(),
            this->dataPtr->lastMeasurementTime);

  msgs::LaserScan *scan = this->dataPtr->laserMsg.mutable_scan();

  // Store the latest laser scans into laserMsg
  msgs::Set(scan->mutable_world_pose(),
      this->dataPtr->pose + this->dataPtr->parentEntity->GetWorldPose().Ign());
  scan->set_angle_min(this->AngleMin().Radian());
  scan->set_angle_max(this->AngleMax().Radian());
  scan->set_angle_step(this->AngleResolution());
  scan->set_count(this->RangeCount());

  scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
  scan->set_vertical_angle_step(this->VerticalAngleResolution());
  scan->set_vertical_count(this->VerticalRangeCount());

  scan->set_range_min(this->RangeMin());
  scan->set_range_max(this->RangeMax());

  scan->clear_ranges();
  scan->clear_intensities();

  unsigned int rayCount = this->RayCount();
  unsigned int rangeCount = this->RangeCount();
  unsigned int verticalRayCount = this->VerticalRayCount();
  unsigned int verticalRangeCount = this->VerticalRangeCount();

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
        r1 = this->LaserShape()->GetRange(j1);
        r2 = this->LaserShape()->GetRange(j2);
        r3 = this->LaserShape()->GetRange(j3);
        r4 = this->LaserShape()->GetRange(j4);
        range = (1-vb)*((1 - hb) * r1 + hb * r2)
            + vb *((1 - hb) * r3 + hb * r4);

        // intensity is averaged
        intensity = 0.25 * (this->LaserShape()->GetRetro(j1)
            + this->LaserShape()->GetRetro(j2)
            + this->LaserShape()->GetRetro(j3)
            + this->LaserShape()->GetRetro(j4));
      }
      else
      {
        range = this->dataPtr->laserShape->GetRange(j * this->RayCount() + i);
        intensity = this->dataPtr->laserShape->GetRetro(j *
            this->RayCount() + i);
      }

      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (range >= this->RangeMax())
      {
        range = IGN_DBL_INF;
      }
      else if (range <= this->RangeMin())
      {
        range = -IGN_DBL_INF;
      }
      else if (this->dataPtr->noises.find(RAY_NOISE) !=
               this->dataPtr->noises.end())
      {
        // currently supports only one noise model per laser sensor
        range = this->dataPtr->noises[RAY_NOISE]->Apply(range);
        range = ignition::math::clamp(range,
            this->RangeMin(), this->RangeMax());
      }

      scan->add_ranges(range);
      scan->add_intensities(intensity);
    }
  }

  if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
    this->dataPtr->scanPub->Publish(this->dataPtr->laserMsg);

  return true;
}

//////////////////////////////////////////////////
bool RaySensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

//////////////////////////////////////////////////
physics::MultiRayShapePtr RaySensor::GetLaserShape() const
{
  return this->LaserShape();
}

//////////////////////////////////////////////////
physics::MultiRayShapePtr RaySensor::LaserShape() const
{
  return this->dataPtr->laserShape;
}
