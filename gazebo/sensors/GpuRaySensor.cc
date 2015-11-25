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
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"

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

//////////////////////////////////////////////////
GpuRaySensor::GpuRaySensor()
    : Sensor(*new GpuRaySensorPrivate, sensors::IMAGE)
{
  this->dataPtr = std::static_pointer_cast<GpuRaySensorPrivate>(this->dPtr);

  this->dataPtr->rendered = false;
  this->dataPtr->active = false;
  this->dataPtr->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&GpuRaySensor::Render, this)));
}

//////////////////////////////////////////////////
GpuRaySensor::~GpuRaySensor()
{
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
}

//////////////////////////////////////////////////
void GpuRaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->dataPtr->scanPub =
    this->dataPtr->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

  sdf::ElementPtr rayElem = this->dataPtr->sdf->GetElement("ray");
  this->dataPtr->scanElem = rayElem->GetElement("scan");
  this->dataPtr->horzElem = this->dataPtr->scanElem->GetElement("horizontal");
  this->dataPtr->rangeElem = rayElem->GetElement("range");

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

  // Handle noise model settings.
  if (rayElem->HasElement("noise"))
  {
    this->dataPtr->noises[GPU_RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->dataPtr->world->GetEntity(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != NULL,
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

  std::string worldName = this->dataPtr->world->GetName();

  if (!worldName.empty())
  {
    this->dataPtr->scene = rendering::get_scene(worldName);

    if (!this->dataPtr->scene)
      this->dataPtr->scene = rendering::create_scene(worldName, false, true);

    this->dataPtr->laserCam = this->dataPtr->scene->CreateGpuLaser(
        this->dataPtr->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->laserCam)
    {
      gzerr << "Unable to create gpu laser sensor\n";
      return;
    }
    this->dataPtr->laserCam->SetCaptureData(true);

    // initialize GpuLaser from sdf
    if (this->dataPtr->vertRayCount == 1)
    {
      this->dataPtr->vertRangeCount = 1;
      this->dataPtr->laserCam->SetIsHorizontal(true);
    }
    else
      this->dataPtr->laserCam->SetIsHorizontal(false);

    this->dataPtr->rangeCountRatio =
      this->dataPtr->horzRangeCount / this->dataPtr->vertRangeCount;

    this->dataPtr->laserCam->SetNearClip(this->RangeMin());
    this->dataPtr->laserCam->SetFarClip(this->RangeMax());

    this->dataPtr->laserCam->SetHorzFOV(
        (this->AngleMax() - this->AngleMin()).Radian());
    this->dataPtr->laserCam->SetVertFOV(
        (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian());

    this->dataPtr->laserCam->SetHorzHalfAngle(
      (this->AngleMax() + this->AngleMin()).Radian() / 2.0);

    this->dataPtr->laserCam->SetVertHalfAngle((this->VerticalAngleMax()
            + this->VerticalAngleMin()).Radian() / 2.0);

    if (this->HorzFOV() > 2 * M_PI)
      this->dataPtr->laserCam->SetHorzFOV(2*M_PI);

    this->dataPtr->laserCam->SetCameraCount(1);

    if (this->HorzFOV() > 2.8)
    {
      if (this->HorzFOV() > 5.6)
        this->dataPtr->laserCam->SetCameraCount(3);
      else
        this->dataPtr->laserCam->SetCameraCount(2);
    }

    this->dataPtr->laserCam->SetHorzFOV(this->HorzFOV() / this->CameraCount());
    this->dataPtr->horzRayCount /= this->CameraCount();

    if (this->VertFOV() > M_PI / 2)
    {
      gzwarn << "Vertical FOV for block GPU laser is capped at 90 degrees.\n";
      this->dataPtr->laserCam->SetVertFOV(M_PI / 2);
      this->SetVerticalAngleMin(this->dataPtr->laserCam->GetVertHalfAngle() -
                                (this->VertFOV() / 2));
      this->SetVerticalAngleMax(this->dataPtr->laserCam->GetVertHalfAngle() +
                                (this->VertFOV() / 2));
    }

    if ((this->dataPtr->horzRayCount * this->dataPtr->vertRayCount) <
        (this->dataPtr->horzRangeCount * this->dataPtr->vertRangeCount))
    {
      this->dataPtr->horzRayCount = std::max(this->dataPtr->horzRayCount, this->dataPtr->horzRangeCount);
      this->dataPtr->vertRayCount = std::max(this->dataPtr->vertRayCount, this->dataPtr->vertRangeCount);
    }

    if (this->dataPtr->laserCam->IsHorizontal())
    {
      if (this->dataPtr->vertRayCount > 1)
      {
        this->dataPtr->laserCam->SetCosHorzFOV(
          2 * atan(tan(this->HorzFOV()/2) / cos(this->VertFOV()/2)));
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
        this->dataPtr->laserCam->SetRayCountRatio(
          tan(this->CosHorzFOV()/2.0) / tan(this->VertFOV()/2.0));

        if ((this->dataPtr->horzRayCount / this->RayCountRatio()) >
            this->dataPtr->vertRayCount)
        {
          this->dataPtr->vertRayCount =
            this->dataPtr->horzRayCount / this->RayCountRatio();
        }
        else
        {
          this->dataPtr->horzRayCount =
            this->dataPtr->vertRayCount * this->RayCountRatio();
        }
      }
      else
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
      }
    }
    else
    {
      if (this->dataPtr->horzRayCount > 1)
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(
          2 * atan(tan(this->VertFOV()/2) / cos(this->HorzFOV()/2)));
        this->dataPtr->laserCam->SetRayCountRatio(
          tan(this->HorzFOV()/2.0) / tan(this->CosVertFOV()/2.0));

        if ((this->dataPtr->horzRayCount / this->RayCountRatio()) >
            this->dataPtr->vertRayCount)
        {
          this->dataPtr->vertRayCount =
            this->dataPtr->horzRayCount / this->RayCountRatio();
        }
        else
        {
          this->dataPtr->horzRayCount = this->dataPtr->vertRayCount *
            this->RayCountRatio();
        }
      }
      else
      {
        this->dataPtr->laserCam->SetCosHorzFOV(this->HorzFOV());
        this->dataPtr->laserCam->SetCosVertFOV(this->VertFOV());
      }
    }

    // Initialize camera sdf for GpuLaser
    this->dataPtr->cameraElem.reset(new sdf::Element);
    sdf::initFile("camera.sdf", this->dataPtr->cameraElem);

    this->dataPtr->cameraElem->GetElement("horizontal_fov")->Set(
        this->CosHorzFOV());

    sdf::ElementPtr ptr = this->dataPtr->cameraElem->GetElement("image");
    ptr->GetElement("width")->Set(this->dataPtr->horzRayCount);
    ptr->GetElement("height")->Set(this->dataPtr->vertRayCount);
    ptr->GetElement("format")->Set("R8G8B8");

    ptr = this->dataPtr->cameraElem->GetElement("clip");
    ptr->GetElement("near")->Set(this->dataPtr->laserCam->GetNearClip());
    ptr->GetElement("far")->Set(this->dataPtr->laserCam->GetFarClip());

    // Load camera sdf for GpuLaser
    this->dataPtr->laserCam->Load(this->dataPtr->cameraElem);


    // initialize GpuLaser
    this->dataPtr->laserCam->Init();
    this->dataPtr->laserCam->SetRangeCount(
        this->dataPtr->horzRangeCount, this->dataPtr->vertRangeCount);
    this->dataPtr->laserCam->SetClipDist(this->RangeMin(), this->RangeMax());
    this->dataPtr->laserCam->CreateLaserTexture(
        this->ScopedName() + "_RttTex_Laser");
    this->dataPtr->laserCam->CreateRenderTexture(
        this->ScopedName() + "_RttTex_Image");
    this->dataPtr->laserCam->SetWorldPose(this->dataPtr->pose);
    this->dataPtr->laserCam->AttachToVisual(this->ParentId(), true);

    this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
  }
  else
    gzerr << "No world name\n";

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->dataPtr->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

//////////////////////////////////////////////////
void GpuRaySensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->scene->RemoveCamera(this->dataPtr->laserCam->GetName());
  this->dataPtr->laserCam.reset();
  this->dataPtr->scene.reset();
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuRaySensor::ConnectNewLaserFrame(
  boost::function<void(const float *, unsigned int, unsigned int, unsigned int,
  const std::string &)> _subscriber)
{
  return this->dataPtr->laserCam->ConnectNewLaserFrame(_subscriber);
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuRaySensor::ConnectNewLaserFrame(
  std::function<void(const float *, unsigned int, unsigned int, unsigned int,
  const std::string &)> _subscriber)
{
  return this->dataPtr->laserCam->ConnectNewLaserFrame(_subscriber);
}

//////////////////////////////////////////////////
void GpuRaySensor::DisconnectNewLaserFrame(event::ConnectionPtr &_conn)
{
  this->dataPtr->laserCam->DisconnectNewLaserFrame(_conn);
}

//////////////////////////////////////////////////
unsigned int GpuRaySensor::GetCameraCount() const
{
  return this->CameraCount();
}

//////////////////////////////////////////////////
unsigned int GpuRaySensor::CameraCount() const
{
  return this->dataPtr->laserCam->GetCameraCount();
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsHorizontal() const
{
  return this->dataPtr->laserCam->IsHorizontal();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHorzHalfAngle() const
{
  return this->dataPtr->laserCam->GetHorzHalfAngle();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVertHalfAngle() const
{
  return this->dataPtr->laserCam->GetVertHalfAngle();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHorzFOV() const
{
  return this->HorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::HorzFOV() const
{
  return this->dataPtr->laserCam->GetHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosHorzFOV() const
{
  return this->CosHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::CosHorzFOV() const
{
  return this->dataPtr->laserCam->GetCosHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVertFOV() const
{
  return this->VertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::VertFOV() const
{
  return this->dataPtr->laserCam->GetVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosVertFOV() const
{
  return this->CosVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::CosVertFOV() const
{
  return this->dataPtr->laserCam->GetCosVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRayCountRatio() const
{
  return this->RayCountRatio();
}

//////////////////////////////////////////////////
double GpuRaySensor::RayCountRatio() const
{
  return this->dataPtr->laserCam->GetRayCountRatio();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeCountRatio() const
{
  return this->RangeCountRatio();
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
double GpuRaySensor::GetRangeMin() const
{
  return this->RangeMin();
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeMin() const
{
  return this->dataPtr->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeMax() const
{
  return this->RangeMax();
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeMax() const
{
  return this->dataPtr->rangeElem->Get<double>("max");
}

/////////////////////////////////////////////////
double GpuRaySensor::GetAngleResolution() const
{
  return this->AngleResolution();
}

/////////////////////////////////////////////////
double GpuRaySensor::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeResolution() const
{
  return this->RangeResolution();
}

//////////////////////////////////////////////////
double GpuRaySensor::RangeResolution() const
{
  return this->dataPtr->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRayCount() const
{
  return this->RayCount();
}

//////////////////////////////////////////////////
int GpuRaySensor::RayCount() const
{
  return this->dataPtr->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRangeCount() const
{
  return this->RangeCount();
}

//////////////////////////////////////////////////
int GpuRaySensor::RangeCount() const
{
  return this->RayCount() * this->dataPtr->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetVerticalRayCount() const
{
  return this->VerticalRayCount();
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
int GpuRaySensor::GetVerticalRangeCount() const
{
  return this->VerticalRangeCount();
}

//////////////////////////////////////////////////
int GpuRaySensor::VerticalRangeCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
  {
    int rows =  (this->VerticalRayCount() *
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
double GpuRaySensor::GetVerticalAngleResolution() const
{
  return this->VerticalAngleResolution();
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
void GpuRaySensor::GetRanges(std::vector<double> &_ranges)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  _ranges.resize(this->dataPtr->laserMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->dataPtr->laserMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->laserMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRange(int _index)
{
  return this->Range(_index);
}

//////////////////////////////////////////////////
double GpuRaySensor::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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

//////////////////////////////////////////////////
double GpuRaySensor::GetRetro(int _index) const
{
  return this->Retro(_index);
}

//////////////////////////////////////////////////
double GpuRaySensor::Retro(const int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int GpuRaySensor::GetFiducial(int /*_index*/) const
{
  return -1;
}

//////////////////////////////////////////////////
void GpuRaySensor::Render()
{
  if (!this->dataPtr->laserCam || !this->IsActive() || !this->NeedsUpdate())
    return;

  this->dataPtr->lastMeasurementTime = this->dataPtr->scene->GetSimTime();

  this->dataPtr->laserCam->Render();
  this->dataPtr->rendered = true;
}

//////////////////////////////////////////////////
bool GpuRaySensor::UpdateImpl(bool /*_force*/)
{
  if (!this->dataPtr->rendered)
    return false;

  this->dataPtr->laserCam->PostRender();

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
  scan->set_count(this->RayCount());

  scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
  scan->set_vertical_angle_step(this->VerticalAngleResolution());
  scan->set_vertical_count(this->VerticalRayCount());

  scan->set_range_min(this->RangeMin());
  scan->set_range_max(this->RangeMax());

  bool add = scan->ranges_size() == 0;

  // todo: add loop for vertical range count
  for (int j = 0; j < this->VerticalRayCount(); ++j)
  {
    for (int i = 0; i < this->RayCount(); ++i)
    {
      int index = j * this->RayCount() + i;
      double range = this->dataPtr->laserCam->GetLaserData()[index * 3];

      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (range >= this->RangeMax())
      {
        range = GZ_DBL_INF;
      }
      else if (range <= this->RangeMin())
      {
        range = -GZ_DBL_INF;
      }
      else if (this->dataPtr->noises.find(GPU_RAY_NOISE) !=
               this->dataPtr->noises.end())
      {
        range = this->dataPtr->noises[GPU_RAY_NOISE]->Apply(range);
        range = ignition::math::clamp(range,
            this->RangeMin(), this->RangeMax());
      }

      range = ignition::math::isnan(range) ? this->RangeMax() : range;

      if (add)
      {
        scan->add_ranges(range);
        scan->add_intensities(
            this->dataPtr->laserCam->GetLaserData()[index * 3 + 1]);
      }
      else
      {
        scan->set_ranges(index, range);
        scan->set_intensities(index,
            this->dataPtr->laserCam->GetLaserData()[index * 3 + 1]);
      }
    }
  }

  if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
    this->dataPtr->scanPub->Publish(this->dataPtr->laserMsg);

  this->dataPtr->rendered = false;

  return true;
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

//////////////////////////////////////////////////
rendering::GpuLaserPtr GpuRaySensor::GetLaserCamera() const
{
  return this->LaserCamera();
}

//////////////////////////////////////////////////
rendering::GpuLaserPtr GpuRaySensor::LaserCamera() const
{
  return this->dataPtr->laserCam;
}
