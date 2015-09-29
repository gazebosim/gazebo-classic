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
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
*/

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/math/Rand.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/GpuLaser.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/GpuRaySensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gpu_ray", GpuRaySensor)

//////////////////////////////////////////////////
GpuRaySensor::GpuRaySensor()
    : Sensor(sensors::IMAGE)
{
  this->rendered = false;
  this->active = false;
  this->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&GpuRaySensor::Render, this)));
}

//////////////////////////////////////////////////
GpuRaySensor::~GpuRaySensor()
{
}

//////////////////////////////////////////////////
std::string GpuRaySensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/scan";
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

  this->scanPub = this->node->Advertise<msgs::LaserScanStamped>(
      this->GetTopic(), 50);

  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  this->scanElem = rayElem->GetElement("scan");
  this->horzElem = this->scanElem->GetElement("horizontal");
  this->rangeElem = rayElem->GetElement("range");

  if (this->scanElem->HasElement("vertical"))
    this->vertElem = this->scanElem->GetElement("vertical");

  this->horzRayCount = this->GetRayCount();
  this->vertRayCount = this->GetVerticalRayCount();

  if (this->horzRayCount == 0 || this->vertRayCount == 0)
  {
    gzthrow("GpuRaySensor: Image has 0 size!");
  }

  this->horzRangeCount = this->GetRangeCount();
  this->vertRangeCount = this->GetVerticalRangeCount();

  // Handle noise model settings.
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
void GpuRaySensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create GpuRaySensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    this->laserCam = this->scene->CreateGpuLaser(
        this->sdf->Get<std::string>("name"), false);

    if (!this->laserCam)
    {
      gzerr << "Unable to create gpu laser sensor\n";
      return;
    }
    this->laserCam->SetCaptureData(true);

    // initialize GpuLaser from sdf
    if (this->vertRayCount == 1)
    {
      this->vertRangeCount = 1;
      this->laserCam->SetIsHorizontal(true);
    }
    else
      this->laserCam->SetIsHorizontal(false);

    this->rangeCountRatio = this->horzRangeCount / this->vertRangeCount;

    this->laserCam->SetNearClip(this->GetRangeMin());
    this->laserCam->SetFarClip(this->GetRangeMax());

    this->laserCam->SetHorzFOV(
      (this->GetAngleMax() - this->GetAngleMin()).Radian());
    this->laserCam->SetVertFOV((this->GetVerticalAngleMax()
            - this->GetVerticalAngleMin()).Radian());

    this->laserCam->SetHorzHalfAngle(
      (this->GetAngleMax() + this->GetAngleMin()).Radian() / 2.0);

    this->laserCam->SetVertHalfAngle((this->GetVerticalAngleMax()
            + this->GetVerticalAngleMin()).Radian() / 2.0);

    if (this->GetHorzFOV() > 2 * M_PI)
      this->laserCam->SetHorzFOV(2*M_PI);

    this->laserCam->SetCameraCount(1);

    if (this->GetHorzFOV() > 2.8)
    {
      if (this->GetHorzFOV() > 5.6)
        this->laserCam->SetCameraCount(3);
      else
        this->laserCam->SetCameraCount(2);
    }

    this->laserCam->SetHorzFOV(this->GetHorzFOV() / this->GetCameraCount());
    this->horzRayCount /= this->GetCameraCount();

    if (this->GetVertFOV() > M_PI / 2)
    {
      gzwarn << "Vertical FOV for block GPU laser is capped at 90 degrees.\n";
      this->laserCam->SetVertFOV(M_PI / 2);
      this->SetVerticalAngleMin(this->laserCam->GetVertHalfAngle() -
                                (this->GetVertFOV() / 2));
      this->SetVerticalAngleMax(this->laserCam->GetVertHalfAngle() +
                                (this->GetVertFOV() / 2));
    }

    if ((this->horzRayCount * this->vertRayCount) <
        (this->horzRangeCount * this->vertRangeCount))
    {
      this->horzRayCount = std::max(this->horzRayCount, this->horzRangeCount);
      this->vertRayCount = std::max(this->vertRayCount, this->vertRangeCount);
    }

    if (this->laserCam->IsHorizontal())
    {
      if (this->vertRayCount > 1)
      {
        this->laserCam->SetCosHorzFOV(
          2 * atan(tan(this->GetHorzFOV()/2) / cos(this->GetVertFOV()/2)));
        this->laserCam->SetCosVertFOV(this->GetVertFOV());
        this->laserCam->SetRayCountRatio(
          tan(this->GetCosHorzFOV()/2.0) / tan(this->GetVertFOV()/2.0));

        if ((this->horzRayCount / this->GetRayCountRatio()) >
            this->vertRayCount)
          this->vertRayCount = this->horzRayCount / this->GetRayCountRatio();
        else
          this->horzRayCount = this->vertRayCount * this->GetRayCountRatio();
      }
      else
      {
        this->laserCam->SetCosHorzFOV(this->GetHorzFOV());
        this->laserCam->SetCosVertFOV(this->GetVertFOV());
      }
    }
    else
    {
      if (this->horzRayCount > 1)
      {
        this->laserCam->SetCosHorzFOV(this->GetHorzFOV());
        this->laserCam->SetCosVertFOV(
          2 * atan(tan(this->GetVertFOV()/2) / cos(this->GetHorzFOV()/2)));
        this->laserCam->SetRayCountRatio(
          tan(this->GetHorzFOV()/2.0) / tan(this->GetCosVertFOV()/2.0));

        if ((this->horzRayCount / this->GetRayCountRatio()) >
            this->vertRayCount)
          this->vertRayCount = this->horzRayCount / this->GetRayCountRatio();
        else
          this->horzRayCount = this->vertRayCount * this->GetRayCountRatio();
      }
      else
      {
        this->laserCam->SetCosHorzFOV(this->GetHorzFOV());
        this->laserCam->SetCosVertFOV(this->GetVertFOV());
      }
    }

    // Initialize camera sdf for GpuLaser
    this->cameraElem.reset(new sdf::Element);
    sdf::initFile("camera.sdf", this->cameraElem);

    this->cameraElem->GetElement("horizontal_fov")->Set(this->GetCosHorzFOV());

    sdf::ElementPtr ptr = this->cameraElem->GetElement("image");
    ptr->GetElement("width")->Set(this->horzRayCount);
    ptr->GetElement("height")->Set(this->vertRayCount);
    ptr->GetElement("format")->Set("R8G8B8");

    ptr = this->cameraElem->GetElement("clip");
    ptr->GetElement("near")->Set(this->laserCam->GetNearClip());
    ptr->GetElement("far")->Set(this->laserCam->GetFarClip());

    // Load camera sdf for GpuLaser
    this->laserCam->Load(this->cameraElem);


    // initialize GpuLaser
    this->laserCam->Init();
    this->laserCam->SetRangeCount(this->horzRangeCount, this->vertRangeCount);
    this->laserCam->SetClipDist(this->GetRangeMin(), this->GetRangeMax());
    this->laserCam->CreateLaserTexture(
        this->GetScopedName() + "_RttTex_Laser");
    this->laserCam->CreateRenderTexture(
        this->GetScopedName() + "_RttTex_Image");
    this->laserCam->SetWorldPose(this->pose);
    this->laserCam->AttachToVisual(this->parentId, true);

    this->laserMsg.mutable_scan()->set_frame(this->parentName);
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
  Sensor::Fini();
  this->scene->RemoveCamera(this->laserCam->GetName());
  this->laserCam.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuRaySensor::ConnectNewLaserFrame(
  boost::function<void(const float *, unsigned int, unsigned int, unsigned int,
  const std::string &)> _subscriber)
{
  return this->laserCam->ConnectNewLaserFrame(_subscriber);
}

//////////////////////////////////////////////////
void GpuRaySensor::DisconnectNewLaserFrame(event::ConnectionPtr &_conn)
{
  this->laserCam->DisconnectNewLaserFrame(_conn);
}

//////////////////////////////////////////////////
unsigned int GpuRaySensor::GetCameraCount() const
{
  return this->laserCam->GetCameraCount();
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsHorizontal() const
{
  return this->laserCam->IsHorizontal();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHorzHalfAngle() const
{
  return this->laserCam->GetHorzHalfAngle();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVertHalfAngle() const
{
  return this->laserCam->GetVertHalfAngle();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHorzFOV() const
{
  return this->laserCam->GetHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosHorzFOV() const
{
  return this->laserCam->GetCosHorzFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVertFOV() const
{
  return this->laserCam->GetVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosVertFOV() const
{
  return this->laserCam->GetCosVertFOV();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRayCountRatio() const
{
  return this->laserCam->GetRayCountRatio();
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeCountRatio() const
{
  return this->rangeCountRatio;
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMin() const
{
  return this->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMin(double _angle)
{
  this->horzElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMax() const
{
  return this->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMax(double _angle)
{
  this->horzElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeMin() const
{
  return this->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeMax() const
{
  return this->rangeElem->Get<double>("max");
}

/////////////////////////////////////////////////
double GpuRaySensor::GetAngleResolution() const
{
  return (this->GetAngleMax() - this->GetAngleMin()).Radian() /
    (this->GetRangeCount()-1);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeResolution() const
{
  return this->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRayCount() const
{
  return this->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRangeCount() const
{
  return this->GetRayCount() *
        this->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetVerticalRayCount() const
{
  if (this->scanElem->HasElement("vertical"))
    return this->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
int GpuRaySensor::GetVerticalRangeCount() const
{
  if (this->scanElem->HasElement("vertical"))
  {
    int rows =  (this->GetVerticalRayCount() *
          this->vertElem->Get<double>("resolution"));
    if (rows > 1)
      return rows;
    else
      return 1;
  }
  else
    return 1;
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetVerticalAngleMin() const
{
  if (this->scanElem->HasElement("vertical"))
    return this->vertElem->Get<double>("min_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
void GpuRaySensor::SetVerticalAngleMin(double _angle)
{
  if (this->scanElem->HasElement("vertical"))
    this->vertElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetVerticalAngleMax() const
{
  if (this->scanElem->HasElement("vertical"))
    return this->vertElem->Get<double>("max_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVerticalAngleResolution() const
{
  return (this->GetVerticalAngleMax() - this->GetVerticalAngleMin()).Radian() /
    (this->GetVerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void GpuRaySensor::SetVerticalAngleMax(double _angle)
{
  if (this->scanElem->HasElement("vertical"))
    this->vertElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void GpuRaySensor::GetRanges(std::vector<double> &_ranges)
{
  boost::mutex::scoped_lock lock(this->mutex);

  _ranges.resize(this->laserMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->laserMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->laserMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRange(int _index)
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (this->laserMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index > this->laserMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->laserMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRetro(int /*_index*/) const
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
  if (!this->laserCam || !this->IsActive() || !this->NeedsUpdate())
    return;

  this->lastMeasurementTime = this->scene->GetSimTime();

  this->laserCam->Render();
  this->rendered = true;
}

//////////////////////////////////////////////////
bool GpuRaySensor::UpdateImpl(bool /*_force*/)
{
  if (!this->rendered)
    return false;

  this->laserCam->PostRender();

  boost::mutex::scoped_lock lock(this->mutex);

  msgs::Set(this->laserMsg.mutable_time(), this->lastMeasurementTime);

  msgs::LaserScan *scan = this->laserMsg.mutable_scan();

  // Store the latest laser scans into laserMsg
  msgs::Set(scan->mutable_world_pose(),
      this->pose + this->parentEntity->GetWorldPose());
  scan->set_angle_min(this->GetAngleMin().Radian());
  scan->set_angle_max(this->GetAngleMax().Radian());
  scan->set_angle_step(this->GetAngleResolution());
  scan->set_count(this->GetRayCount());

  scan->set_vertical_angle_min(this->GetVerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->GetVerticalAngleMax().Radian());
  scan->set_vertical_angle_step(this->GetVerticalAngleResolution());
  scan->set_vertical_count(this->GetVerticalRayCount());

  scan->set_range_min(this->GetRangeMin());
  scan->set_range_max(this->GetRangeMax());

  bool add = scan->ranges_size() == 0;

  // todo: add loop for vertical range count
  for (int j = 0; j < this->GetVerticalRayCount(); ++j)
  {
    for (int i = 0; i < this->GetRayCount(); ++i)
    {
      int index = j * this->GetRayCount() + i;
      double range = this->laserCam->GetLaserData()[index * 3];

      if (!this->noises.empty())
      {
        range = this->noises[0]->Apply(range);
        range = math::clamp(range, this->GetRangeMin(), this->GetRangeMax());
      }

      range = math::isnan(range) ? this->GetRangeMax() : range;

      if (add)
      {
        scan->add_ranges(range);
        scan->add_intensities(this->laserCam->GetLaserData()[index * 3 + 1]);
      }
      else
      {
        scan->set_ranges(index, range);
        scan->set_intensities(index,
            this->laserCam->GetLaserData()[index * 3 + 1]);
      }
    }
  }

  if (this->scanPub && this->scanPub->HasConnections())
    this->scanPub->Publish(this->laserMsg);

  this->rendered = false;

  return true;
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->scanPub && this->scanPub->HasConnections());
}
