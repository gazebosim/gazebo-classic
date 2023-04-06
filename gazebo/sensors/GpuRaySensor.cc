/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <ignition/common/Profiler.hh>
#include <functional>
#include <utility>
#include <ignition/math.hh>
#include <ignition/math/Helpers.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/GpuLaser.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/GpuRaySensorPrivate.hh"
#include "gazebo/sensors/GpuRaySensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gpu_ray", GpuRaySensor)

void RegisterGpuLidarSensor()
{
  SensorFactory::RegisterSensor("gpu_lidar", NewGpuRaySensor);
}

//////////////////////////////////////////////////
GpuRaySensor::GpuRaySensor()
: Sensor(sensors::IMAGE),
  dataPtr(new GpuRaySensorPrivate)
{
  this->dataPtr->rendered = false;
  this->active = false;
  this->connections.push_back(
      event::Events::ConnectRender(
        std::bind(&GpuRaySensor::Render, this)));
}

//////////////////////////////////////////////////
GpuRaySensor::~GpuRaySensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
std::string GpuRaySensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void GpuRaySensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
  // useStrictRate is set in Sensor::Load()
  if (GpuRaySensor::useStrictRate)
  {
    this->connections.push_back(
        event::Events::ConnectPreRenderEnded(
          boost::bind(&GpuRaySensor::PrerenderEnded, this)));
  }
}

//////////////////////////////////////////////////
void GpuRaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  // useStrictRate is set in Sensor::Load()
  if (GpuRaySensor::useStrictRate)
  {
    this->connections.push_back(
        event::Events::ConnectPreRenderEnded(
          boost::bind(&GpuRaySensor::PrerenderEnded, this)));
  }


  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  this->dataPtr->scanElem = rayElem->GetElement("scan");
  this->dataPtr->horzElem = this->dataPtr->scanElem->GetElement("horizontal");
  this->dataPtr->rangeElem = rayElem->GetElement("range");

  if (this->sdf->HasElement("sample_sensor"))
  {
    gzmsg << "Sample sensor function is activated for sensor\n";
    this->dataPtr->isSampleSensor = this->sdf->GetElement("sample_sensor")->Get<bool>();
    this->dataPtr->sampleSize = this->sdf->GetElement("sample_size")->Get<unsigned int>();
    this->dataPtr->sampleFile = this->sdf->GetElement("sample_csv_file")->Get<std::string>();
  }

  if (!IsSampleSensor())
  {
    this->dataPtr->scanPub =
      this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);
  }
  else
  {
    this->dataPtr->scanPub =
      this->node->Advertise<msgs::LaserScanAnglesStamped>(this->Topic(), 50);
  }
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem = this->dataPtr->scanElem->GetElement("vertical");

  this->dataPtr->horzRayCount = this->RayCount();
  this->dataPtr->vertRayCount = this->VerticalRayCount();

  if (this->dataPtr->horzRayCount == 0 || this->dataPtr->vertRayCount == 0)
  {
    gzthrow("GpuRaySensor: Image has 0 size!");
  }

  this->dataPtr->horzRangeCount = this->RangeCount();
  this->dataPtr->vertRangeCount = this->VerticalRangeCount();

  this->dataPtr->rangeMin = this->RangeMin();
  this->dataPtr->rangeMax = this->RangeMax();

  // Handle noise model settings.
  if (rayElem->HasElement("noise"))
  {
    this->noises[GPU_RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->world->EntityByName(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != nullptr,
      "Unable to get the parent entity.");
}

//////////////////////////////////////////////////
void GpuRaySensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create GpuRaySensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->Name();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    if (!this->scene)
    {
      gzerr << "Unable to create gpu laser sensor: no scene\n";
      return;
    }
    this->dataPtr->laserCam = this->scene->CreateGpuLaser(
        this->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->laserCam)
    {
      gzerr << "Unable to create gpu laser sensor\n";
      return;
    }
    this->dataPtr->laserCam->SetCaptureData(true);

    // initialize GpuLaser from sdf
    // assume horizontal sweep (rotation around z axis) like cpu ray sensor
    this->dataPtr->laserCam->SetIsHorizontal(true);

    this->dataPtr->laserCam->SetNearClip(this->RangeMin());
    this->dataPtr->laserCam->SetFarClip(this->RangeMax());

    // horizontal laser setup
    double hfovTotal = (this->AngleMax() - this->AngleMin()).Radian();

    if (hfovTotal > 2 * M_PI)
    {
      hfovTotal = 2 * M_PI;
      gzwarn << "Horizontal FOV for GPU laser is capped at 360 degrees.\n";
    }

    this->dataPtr->laserCam->SetHorzHalfAngle(this->HorzHalfAngle());

    // we use a fixed square camera FOV
    constexpr double hfovPerCamera = M_PI_2;
    this->dataPtr->laserCam->SetHorzFOV(hfovPerCamera);

    // vertical laser setup
    double vfovTotal;

    if (this->dataPtr->vertRayCount > 1)
    {
      vfovTotal = (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian();
    }
    else
    {
      vfovTotal = 0;

      if (this->VerticalAngleMax() != this->VerticalAngleMin())
      {
        gzwarn << "Only one vertical ray but vertical min. and max. angle "
                  "are not equal. Half angle between min. and max. is used.\n";
        const double vertHalfAngle = this->VertHalfAngle();
        this->SetVerticalAngleMin(vertHalfAngle);
        this->SetVerticalAngleMax(vertHalfAngle);
      }
    }

    if (vfovTotal > M_PI)
    {
      vfovTotal = M_PI;
      gzwarn << "Vertical FOV for GPU laser is capped at 180 degrees.\n";
    }

    constexpr double vfovPerCamera = M_PI_2;
    this->dataPtr->laserCam->SetVertFOV(vfovPerCamera);
    this->dataPtr->laserCam->SetVertHalfAngle(this->VertHalfAngle());

    // unused by this implementation, but keep for backwards compatibility
    const double cosHorzFov =
        2 * atan(tan(hfovPerCamera / 2) / cos(vfovTotal / 2));
    const double cosVertFov =
        2 * atan(tan(vfovTotal / 2) / cos(hfovPerCamera / 2));
    this->dataPtr->laserCam->SetCosHorzFOV(cosHorzFov);
    this->dataPtr->laserCam->SetCosVertFOV(cosVertFov);

    // internal camera has fixed aspect ratio of one
    constexpr double cameraAspectRatio = 1;
    this->dataPtr->laserCam->SetRayCountRatio(cameraAspectRatio);

    // If vertical ray is not 1 adjust horizontal and vertical
    // ray count to maintain aspect ratio
    if (this->dataPtr->vertRayCount > 1)
    {
      this->dataPtr->rangeCountRatio = cameraAspectRatio;
    }

    // take ranges per radian of FOV as a guideline for camera resolution
    double rangesPerFov = 0;
    if (vfovTotal > 0)
    {
      rangesPerFov = std::max(rangesPerFov, this->VerticalRangeCount() / vfovTotal);
    }
    if (hfovTotal > 0)
    {
      rangesPerFov = std::max(rangesPerFov, this->RangeCount() / hfovTotal);
    }

    // ranges per camera (which has 90 deg FOV)
    const unsigned int ranges = static_cast<int>(rangesPerFov * M_PI_2);

    // ensure minimal texture size (to mitigate issues with stepped point cloud
    // especially for shallow angles of incidence)
    constexpr unsigned int min_texture_size = 1024;
    const unsigned int camera_resolution = std::max(ranges, min_texture_size); //TODO GEORG: 

    // Initialize camera sdf for GpuLaser
    this->dataPtr->cameraElem.reset(new sdf::Element);
    sdf::initFile("camera.sdf", this->dataPtr->cameraElem);

    this->dataPtr->cameraElem->GetElement("horizontal_fov")->Set(M_PI_2);

    sdf::ElementPtr ptr = this->dataPtr->cameraElem->GetElement("image");
    ptr->GetElement("width")->Set(camera_resolution);
    ptr->GetElement("height")->Set(camera_resolution);

    if (IsSampleSensor())
    {
      ptr->GetElement("format")->Set("FLOAT16");
    }
    else
    {
      ptr->GetElement("format")->Set("FLOAT32");
    }

    ptr = this->dataPtr->cameraElem->GetElement("clip");
    ptr->GetElement("near")->Set(this->dataPtr->laserCam->NearClip());
    ptr->GetElement("far")->Set(this->dataPtr->laserCam->FarClip());

    // Load camera sdf for GpuLaser
    this->dataPtr->laserCam->Load(this->dataPtr->cameraElem);

    // initialize GpuLaser
    this->dataPtr->laserCam->Init();
    this->dataPtr->laserCam->SetRangeCount(
        this->RangeCount(),
        this->VerticalRangeCount());
    this->dataPtr->laserCam->SetClipDist(static_cast<float>(this->RangeMin()), static_cast<float>(this->RangeMax()));
    if (!IsSampleSensor())
    {
      // create sets of angles and initialize cubemap
      // eventually, this should also be able to handle irregular spaced rays
      // but that would require changes to the SDFormat definition of a ray sensor.
      // Note: The order of the angles in the two sets matters as the laser
      // readings will be returned in the same order!
      std::set<double> azimuth_angles;
      const double azimuth_angle_increment = hfovTotal / (this->dataPtr->horzRangeCount - 1);
      double azimuth = this->AngleMin().Radian();
      for (unsigned int i = 0; i < this->dataPtr->horzRangeCount; i++)
      {
        azimuth_angles.insert(azimuth);
        azimuth += azimuth_angle_increment;
      }

      std::set<double> elevation_angles;
      const double elevation_angle_increment = vfovTotal / (this->dataPtr->vertRangeCount - 1);
      double elevation = this->VerticalAngleMin().Radian();
      for (unsigned int i = 0; i < this->dataPtr->vertRangeCount; i++)
      {
        elevation_angles.insert(elevation);
        elevation += elevation_angle_increment;
      }

      this->dataPtr->laserCam->InitMapping(azimuth_angles, elevation_angles);
      this->dataPtr->laserCam->CreateLaserTexture(
          this->ScopedName() + "_RttTex_Laser", Ogre::PF_FLOAT32_RGB);
    }
    else
    {
      this->dataPtr->laserCam->SetIsSample(true);
      this->dataPtr->laserCam->SetSampleSize(this->dataPtr->sampleSize);

      gzmsg << "load sample csv file name: " << this->dataPtr->sampleFile << "\n";

      std::vector<std::vector<double>> samples;
      if (!readCsvFile(this->dataPtr->sampleFile, samples)) {
          gzthrow("GpuRaySensor: Can not load csv file!");
      }

      std::vector<double> azimuth_angles;
      azimuth_angles.reserve(samples.size());
      std::vector<double> elevation_angles;
      elevation_angles.reserve(samples.size());

      constexpr double deg_2_rad = M_PI / 180.0;
      for (auto sample : samples)
      {
        azimuth_angles.push_back(sample[1] * deg_2_rad);
        elevation_angles.push_back(M_PI_2 - sample[2] * deg_2_rad);
      }

      this->dataPtr->laserCam->InitMapping(azimuth_angles, elevation_angles);
      this->dataPtr->laserCam->CreateLaserTexture(
          this->ScopedName() + "_RttTex_Laser", Ogre::PF_FLOAT16_R);
    }
    this->dataPtr->laserCam->CreateRenderTexture(
        this->ScopedName() + "_RttTex_Image");
    this->dataPtr->laserCam->SetWorldPose(this->pose);
    this->dataPtr->laserCam->AttachToVisual(this->ParentId(), true, 0, 0);
    if (IsSampleSensor())
    {
      this->dataPtr->laserAnglesMsg.mutable_scan()->set_frame(this->ParentName());
    }
    else
    {
      this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
    }
  }
  else
    gzerr << "No world name\n";

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

//////////////////////////////////////////////////
void GpuRaySensor::Fini()
{
  this->dataPtr->scanPub.reset();

  if (this->dataPtr->laserCam)
  {
    this->scene->RemoveCamera(this->dataPtr->laserCam->Name());
  }

  this->dataPtr->laserCam.reset();

  Sensor::Fini();
}

//////////////////////////////////////////////////
void GpuRaySensor::SetActive(bool _value)
{
  // If this sensor is reactivated
  if (GpuRaySensor::useStrictRate && _value && !this->IsActive())
  {
    // the next rendering time must be reset to ensure it is properly
    // computed by GpuRaySensor::NeedsUpdate.
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
  }
  Sensor::SetActive(_value);
}

//////////////////////////////////////////////////
bool GpuRaySensor::NeedsUpdate()
{
  if (GpuRaySensor::useStrictRate)
  {
    double simTime;
    if (this->scene)
      simTime = this->scene->SimTime().Double();
    else
      simTime = this->world->SimTime().Double();

    if (simTime < this->lastMeasurementTime.Double())
    {
      // Rendering sensors also set the lastMeasurementTime variable in Render()
      // and lastUpdateTime in Sensor::Update based on Scene::SimTime() which
      // could be outdated when the world is reset. In this case reset
      // the variables back to 0.
      this->ResetLastUpdateTime();
      return false;
    }

    double dt = this->world->Physics()->GetMaxStepSize();

    // If next rendering time is not set yet
    if (std::isnan(this->dataPtr->nextRenderingTime))
    {
      if (this->updatePeriod == 0
          || (simTime > 0.0 &&
          std::abs(std::fmod(simTime, this->updatePeriod.Double())) < dt))
      {
        this->dataPtr->nextRenderingTime = simTime;
        return true;
      }
      else
      {
        return false;
      }
    }

    if (simTime > this->dataPtr->nextRenderingTime + dt)
      return true;

    // Trigger on the tick the closest from the targeted rendering time
    return (ignition::math::lessOrNearEqual(
          std::abs(simTime - this->dataPtr->nextRenderingTime), dt / 2.0));
  }
  else
  {
    return Sensor::NeedsUpdate();
  }
}

//////////////////////////////////////////////////
void GpuRaySensor::Update(bool _force)
{
  Sensor::Update(_force);
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuRaySensor::ConnectNewLaserFrame(
  std::function<void(const float *, unsigned int, unsigned int, unsigned int,
  const std::string &)> _subscriber)
{
  return this->dataPtr->laserCam->ConnectNewLaserFrame(_subscriber);
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsHorizontal() const
{
  return this->dataPtr->laserCam->IsHorizontal();
}

//////////////////////////////////////////////////
double GpuRaySensor::HorzFOV() const
{
  return this->dataPtr->laserCam->HorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::CosHorzFOV() const
{
  return this->dataPtr->laserCam->CosHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::VertFOV() const
{
  return this->dataPtr->laserCam->VertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::CosVertFOV() const
{
  return this->dataPtr->laserCam->CosVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::RayCountRatio() const
{
  return this->dataPtr->laserCam->RayCountRatio();
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeCountRatio() const
{
  return this->dataPtr->rangeCountRatio;
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRaySensor::AngleMin() const
{
  return this->dataPtr->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMin(double _angle)
{
  this->dataPtr->horzElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRaySensor::AngleMax() const
{
  return this->dataPtr->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMax(double _angle)
{
  this->dataPtr->horzElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeMin() const
{
  return this->dataPtr->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeMax() const
{
  return this->dataPtr->rangeElem->Get<double>("max");
}

/////////////////////////////////////////////////
double GpuRaySensor::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeResolution() const
{
  return this->dataPtr->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::RayCount() const
{
  return this->dataPtr->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
int GpuRaySensor::RangeCount() const
{
  return static_cast<int>(this->RayCount() * this->dataPtr->horzElem->Get<double>("resolution"));
}

//////////////////////////////////////////////////
int GpuRaySensor::VerticalRayCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
int GpuRaySensor::VerticalRangeCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
  {
    const int rows = static_cast<int>(this->VerticalRayCount() *
          this->dataPtr->vertElem->Get<double>("resolution"));
    if (rows > 1)
      return rows;
    else
      return 1;
  }
  else
    return 1;
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRaySensor::VerticalAngleMin() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("min_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
void GpuRaySensor::SetVerticalAngleMin(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle GpuRaySensor::VerticalAngleMax() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("max_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
double GpuRaySensor::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void GpuRaySensor::SetVerticalAngleMax(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void GpuRaySensor::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!IsSampleSensor())
  {
    _ranges.resize(this->dataPtr->laserMsg.scan().ranges_size());
    memcpy(&_ranges[0], this->dataPtr->laserMsg.scan().ranges().data(),
          sizeof(_ranges[0]) * this->dataPtr->laserMsg.scan().ranges_size());
  }
  else
  {
    _ranges.resize(this->dataPtr->laserAnglesMsg.scan().ranges_size());
    memcpy(&_ranges[0], this->dataPtr->laserAnglesMsg.scan().ranges().data(),
          sizeof(_ranges[0]) * this->dataPtr->laserAnglesMsg.scan().ranges_size());
  }
}

//////////////////////////////////////////////////
double GpuRaySensor::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (!IsSampleSensor())
  {
    if (this->dataPtr->laserMsg.scan().ranges_size() == 0)
    {
      gzwarn << "ranges not constructed yet (zero sized)\n";
      return 0.0;
    }
    if (_index < 0 || _index > this->dataPtr->laserMsg.scan().ranges_size())
    {
      gzerr << "Invalid range index[" << _index << "]\n";
      return 0.0;
    }

    return this->dataPtr->laserMsg.scan().ranges(_index);
  }
  else
  {
    if (this->dataPtr->laserAnglesMsg.scan().ranges_size() == 0)
    {
      gzwarn << "ranges not constructed yet (zero sized)\n";
      return 0.0;
    }
    if (_index < 0 || _index > this->dataPtr->laserAnglesMsg.scan().ranges_size())
    {
      gzerr << "Invalid range index[" << _index << "]\n";
      return 0.0;
    }

    return this->dataPtr->laserAnglesMsg.scan().ranges(_index);
  }
}

//////////////////////////////////////////////////
double GpuRaySensor::Retro(const int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int GpuRaySensor::Fiducial(const unsigned int /*_index*/) const
{
  return -1;
}

//////////////////////////////////////////////////
void GpuRaySensor::PrerenderEnded()
{
  if (GpuRaySensor::useStrictRate && this->dataPtr->laserCam && this->IsActive() &&
      this->NeedsUpdate())
  {
    // compute next rendering time, take care of the case where period is zero.
    double dt;
    if (this->updatePeriod <= 0.0)
      dt = this->world->Physics()->GetMaxStepSize();
    else
      dt = this->updatePeriod.Double();
    this->dataPtr->nextRenderingTime += dt;

    this->dataPtr->renderNeeded = true;
    this->lastMeasurementTime = this->scene->SimTime();
  }
}

//////////////////////////////////////////////////
void GpuRaySensor::Render()
{
  IGN_PROFILE("sensors::GpuRaySensor::Render");
  if (GpuRaySensor::useStrictRate)
  {
    if (!this->dataPtr->renderNeeded)
      return;
    this->dataPtr->laserCam->Render();
    this->dataPtr->rendered = true;
    this->dataPtr->renderNeeded = false;
  }
  else
  {
    if (!this->dataPtr->laserCam || !this->IsActive() || !this->NeedsUpdate())
      return;

    this->lastMeasurementTime = this->scene->SimTime();
    this->dataPtr->laserCam->Render();
    this->dataPtr->rendered = true;
  }
}

//////////////////////////////////////////////////
bool GpuRaySensor::UpdateImpl(const bool /*_force*/)
{
  IGN_PROFILE("GpuRaySensor::UpdateImpl");

  if (!this->dataPtr->rendered)
    return false;
  IGN_PROFILE_BEGIN("PostRender");
  this->dataPtr->laserCam->PostRender();
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("fillarray");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  {
    if (!IsSampleSensor())
    {
      // Laser Msg without angles
      msgs::Set(this->dataPtr->laserMsg.mutable_time(),
          this->lastMeasurementTime);

      msgs::LaserScan *scan = this->dataPtr->laserMsg.mutable_scan();

      // Store the latest laser scans into laserMsg
      msgs::Set(scan->mutable_world_pose(),
          this->pose + this->dataPtr->parentEntity->WorldPose());
      scan->set_angle_min(this->AngleMin().Radian());
      scan->set_angle_max(this->AngleMax().Radian());
      scan->set_angle_step(this->AngleResolution());
      scan->set_count(this->RangeCount());

      scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
      scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
      scan->set_vertical_angle_step(this->VerticalAngleResolution());
      scan->set_vertical_count(this->dataPtr->vertRangeCount);

      scan->set_range_min(this->dataPtr->rangeMin);
      scan->set_range_max(this->dataPtr->rangeMax);

      const int numRays = static_cast<int>(this->dataPtr->vertRangeCount *
        this->dataPtr->horzRangeCount);
      if (scan->ranges_size() != numRays)
      {
        // gzdbg << "Size mismatch; allocating memory\n";
        scan->clear_ranges();
        scan->clear_intensities();
        for (int i = 0; i < numRays; ++i)
        {
          scan->add_ranges(ignition::math::NAN_F);
          scan->add_intensities(ignition::math::NAN_F);
        }
      }

      auto dataIter = this->dataPtr->laserCam->LaserDataBegin();
      auto dataEnd = this->dataPtr->laserCam->LaserDataEnd();
      for (int i = 0; dataIter != dataEnd; ++dataIter, ++i)
      {
        const rendering::GpuLaserData data = *dataIter;
        double range = data.range;
        double intensity = data.intensity;

        // Mask ranges outside of min/max to +/- inf, as per REP 117
        if (range >= this->dataPtr->rangeMax)
        {
          range = ignition::math::INF_D;
        }
        else if (range <= this->dataPtr->rangeMin)
        {
          range = -ignition::math::INF_D;
        }
        else if (this->noises.find(GPU_RAY_NOISE) != this->noises.end())
        {
          range = this->noises[GPU_RAY_NOISE]->Apply(range);
          range = ignition::math::clamp(range,
              this->dataPtr->rangeMin, this->dataPtr->rangeMax);
        }

        range = ignition::math::isnan(range) ? this->dataPtr->rangeMax : range;
        scan->set_ranges(i, range);
        scan->set_intensities(i, intensity);
      }

      if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
        this->dataPtr->scanPub->Publish(this->dataPtr->laserMsg);
    }
    else
    {
      // Laser Msg with angles
      msgs::Set(this->dataPtr->laserAnglesMsg.mutable_time(),
          this->lastMeasurementTime);

      msgs::LaserScanAngles *scan = this->dataPtr->laserAnglesMsg.mutable_scan();
      // Store the latest laser scans into laserAnglesMsg
      msgs::Set(scan->mutable_world_pose(),
          this->pose + this->dataPtr->parentEntity->WorldPose());
      scan->set_angle_min(this->AngleMin().Radian());
      scan->set_angle_max(this->AngleMax().Radian());
      scan->set_angle_step(this->AngleResolution());
      scan->set_count(this->RangeCount());

      scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
      scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
      scan->set_vertical_angle_step(this->VerticalAngleResolution());
      scan->set_vertical_count(this->dataPtr->vertRangeCount);

      scan->set_range_min(this->dataPtr->rangeMin);
      scan->set_range_max(this->dataPtr->rangeMax);

      const int numRays = static_cast<int>(this->dataPtr->sampleSize);
      if (scan->ranges_size() != numRays)
      {
        // gzdbg << "Size mismatch; allocating memory\n";
        scan->clear_ranges();
        scan->clear_intensities();
        scan->clear_azimuth();
        scan->clear_zenith();
        for (int i = 0; i < numRays; ++i)
        {
          scan->add_ranges(ignition::math::NAN_F);
          scan->add_intensities(ignition::math::NAN_F);
          scan->add_azimuth(ignition::math::NAN_F);
          scan->add_zenith(ignition::math::NAN_F);
        }
      }

      // ToDo Georg replace!
      std::vector<float>* data = this->dataPtr->laserCam->LaserData();
      constexpr double fixed_intensity = 1.0;

      constexpr unsigned int skip = 3;
      for (unsigned int i = 0; i < data->size() / skip; ++i)
      {
        double range = data->at(i * skip);
        if (range >= this->dataPtr->rangeMax)
        {
          range = ignition::math::INF_D;
        }
        else if (range <= this->dataPtr->rangeMin)
        {
          range = -ignition::math::INF_D;
        }
        else if (this->noises.find(GPU_RAY_NOISE) != this->noises.end())
        {
          range = this->noises[GPU_RAY_NOISE]->Apply(range);
          range = ignition::math::clamp(range,
              this->dataPtr->rangeMin, this->dataPtr->rangeMax);
        }
        double azimuth = data->at(i * skip + 1);
        double zenith = data->at(i * skip + 2);
        range = ignition::math::isnan(range) ? this->dataPtr->rangeMax : range;
        scan->set_ranges(i, range);
        scan->set_intensities(i, fixed_intensity);
        scan->set_azimuth(i, azimuth);
        scan->set_zenith(i, zenith);
      }

      if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
        this->dataPtr->scanPub->Publish(this->dataPtr->laserAnglesMsg);
    }
  }

  this->dataPtr->rendered = false;
  IGN_PROFILE_END();
  return true;
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

//////////////////////////////////////////////////
rendering::GpuLaserPtr GpuRaySensor::LaserCamera() const
{
  return this->dataPtr->laserCam;
}

//////////////////////////////////////////////////
double GpuRaySensor::NextRequiredTimestamp() const
{
  if (GpuRaySensor::useStrictRate)
  {
    if (!ignition::math::equal(this->updatePeriod.Double(), 0.0))
      return this->dataPtr->nextRenderingTime;
    else
      return std::numeric_limits<double>::quiet_NaN();
  }
  else
  {
    return Sensor::NextRequiredTimestamp();
  }
}

//////////////////////////////////////////////////
void GpuRaySensor::ResetLastUpdateTime()
{
  Sensor::ResetLastUpdateTime();
  if (GpuRaySensor::useStrictRate)
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
}

//////////////////////////////////////////////////
unsigned int GpuRaySensor::CameraCount() const
{
  return this->dataPtr->laserCam->CameraCount();
}

//////////////////////////////////////////////////
double GpuRaySensor::HorzHalfAngle() const
{
  return (this->AngleMax() + this->AngleMin()).Radian() / 2.0;
}

//////////////////////////////////////////////////
double GpuRaySensor::VertHalfAngle() const
{
  return (this->VerticalAngleMax() + this->VerticalAngleMin()).Radian() / 2.0;
}

//////////////////////////////////////////////////
bool GpuRaySensor::readCsvFile(std::string file_name, std::vector<std::vector<double>>& datas)
{
  std::fstream file_stream;
  file_stream.open(file_name, std::ios::in);
  if (file_stream.is_open()) {
      std::string header;
      std::getline(file_stream, header, '\n');
      while (!file_stream.eof()) {
          std::string line_str;
          std::getline(file_stream, line_str, '\n');
          std::stringstream line_stream;
          line_stream << line_str;
          std::vector<double> data;
          try {
              while (!line_stream.eof()) {
                  std::string value;
                  std::getline(line_stream, value, ',');
                  data.push_back(std::stod(value));
              }
          } catch (...) {
              continue;
          }
          datas.push_back(data);
      }
      std::cerr << "data size:" << datas.size() << "\n";
      return true;
  } else {
      std::cerr << "cannot read csv file!" << file_name << "\n";
  }
  return false;
}

//////////////////////////////////////////////////
unsigned int GpuRaySensor::SampleSize() const
{
  return this->dataPtr->sampleSize;
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsSampleSensor() const
{
  return this->dataPtr->isSampleSensor;
}
