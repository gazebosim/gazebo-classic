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
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
*/

#include "gazebo/physics/World.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Rendering.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/GpuRaySensor.hh"
#include "gazebo/rendering/GpuLaser.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gpu_ray", GpuRaySensor)

//////////////////////////////////////////////////
GpuRaySensor::GpuRaySensor()
    : Sensor(sensors::IMAGE)
{
  this->active = false;
}

//////////////////////////////////////////////////
GpuRaySensor::~GpuRaySensor()
{
}

//////////////////////////////////////////////////
void GpuRaySensor::Load(const std::string &_worldName, sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
void GpuRaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

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

  if (this->vertRayCount == 1)
  {
    this->vertRangeCount = 1;
    this->isHorizontal = true;
  }
  else
    this->isHorizontal = false;

  this->rangeCountRatio = this->horzRangeCount / this->vertRangeCount;

  this->near = this->GetRangeMin();
  this->far = this->GetRangeMax();

  this->hfov = (this->GetAngleMax() - this->GetAngleMin()).Radian();
  this->vfov = (this->GetVerticalAngleMax()
          - this->GetVerticalAngleMin()).Radian();

  this->horzHalfAngle =
    (this->GetAngleMax() + this->GetAngleMin()).Radian() / 2.0;

  this->vertHalfAngle = (this->GetVerticalAngleMax()
          + this->GetVerticalAngleMin()).Radian() / 2.0;

  if (this->hfov > 2 * M_PI)
    this->hfov = 2*M_PI;

  this->cameraCount = 1;

  if (this->hfov > 2.8)
  {
    if (this->hfov > 5.6)
      this->cameraCount = 3;
    else
      this->cameraCount = 2;
  }

  this->hfov /= this->cameraCount;
  this->horzRayCount /= this->cameraCount;

  if (this->vfov > M_PI / 2)
  {
    gzwarn << "Vertical FOV for block GPU laser is capped at 90 degrees.\n";
    this->vfov = M_PI / 2;
    this->SetVerticalAngleMin(this->vertHalfAngle - (this->vfov / 2));
    this->SetVerticalAngleMax(this->vertHalfAngle + (this->vfov / 2));
  }

  if ((this->horzRayCount * this->vertRayCount) <
      (this->horzRangeCount * this->vertRangeCount))
  {
    this->horzRayCount = std::max(this->horzRayCount, this->horzRangeCount);
    this->vertRayCount = std::max(this->vertRayCount, this->vertRangeCount);
  }

  if (this->isHorizontal)
  {
    if (this->vertRayCount > 1)
    {
      this->chfov = 2 * atan(tan(this->hfov/2) / cos(this->vfov/2));
      this->cvfov = this->vfov;
      this->rayCountRatio = tan(this->chfov/2.0) / tan(this->vfov/2.0);

      if ((this->horzRayCount / this->rayCountRatio) > this->vertRayCount)
        this->vertRayCount = this->horzRayCount / this->rayCountRatio;
      else
        this->horzRayCount = this->vertRayCount * this->rayCountRatio;
    }
    else
    {
      this->chfov = this->hfov;
      this->cvfov = this->vfov;
    }
  }
  else
  {
    if (this->horzRayCount > 1)
    {
      this->chfov = this->hfov;
      this->cvfov = 2 * atan(tan(this->vfov/2) / cos(this->hfov/2));
      this->rayCountRatio = tan(this->hfov/2.0) / tan(this->cvfov/2.0);

      if ((this->horzRayCount / this->rayCountRatio) > this->vertRayCount)
        this->vertRayCount = this->horzRayCount / this->rayCountRatio;
      else
        this->horzRayCount = this->vertRayCount * this->rayCountRatio;
    }
    else
    {
      this->chfov = this->hfov;
      this->cvfov = this->vfov;
    }
  }

  this->cameraElem.reset(new sdf::Element);
  sdf::initFile("camera.sdf", this->cameraElem);

  this->cameraElem->GetElement("horizontal_fov")->Set(this->chfov);

  sdf::ElementPtr ptr = this->cameraElem->GetElement("image");
  ptr->GetElement("width")->Set(this->horzRayCount);
  ptr->GetElement("height")->Set(this->vertRayCount);
  ptr->GetElement("format")->Set("R8G8B8");

  ptr = this->cameraElem->GetElement("clip");
  ptr->GetElement("near")->Set(this->near);
  ptr->GetElement("far")->Set(this->far);
}

//////////////////////////////////////////////////
void GpuRaySensor::Init()
{
  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false);

    this->laserCam = this->scene->CreateGpuLaser(
        this->sdf->GetValueString("name"), false);

    if (!this->laserCam)
    {
      gzerr << "Unable to create gpu laser sensor\n";
      return;
    }
    this->laserCam->SetCaptureData(true);

    this->laserCam->Load(this->cameraElem);

    this->laserCam->Init();
    this->laserCam->SetRangeCount(this->horzRangeCount, this->vertRangeCount);
    this->laserCam->SetClipDist(this->GetRangeMin(), this->GetRangeMax());
    this->laserCam->SetParentSensor(this);
    this->laserCam->CreateLaserTexture(this->GetName() + "_RttTex_Laser");
    this->laserCam->CreateRenderTexture(this->GetName() + "_RttTex_Image");
    this->laserCam->SetWorldPose(this->pose);
    this->laserCam->AttachToVisual(this->parentName, true);
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
}

//////////////////////////////////////////////////
void GpuRaySensor::Fini()
{
  Sensor::Fini();
  this->laserCam->Fini();
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
  return this->cameraCount;
}

//////////////////////////////////////////////////
bool GpuRaySensor::IsHorizontal() const
{
  return this->isHorizontal;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHorzFOV() const
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosHorzFOV() const
{
  return this->chfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVertFOV() const
{
  return this->vfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCosVertFOV() const
{
  return this->cvfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRayCountRatio() const
{
  return this->rayCountRatio;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeCountRatio() const
{
  return this->rangeCountRatio;
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMin() const
{
  return this->horzElem->GetValueDouble("min_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMin(double _angle)
{
  this->horzElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMax() const
{
  return this->horzElem->GetValueDouble("max_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMax(double _angle)
{
  this->horzElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeMin() const
{
  return this->rangeElem->GetValueDouble("min");
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRangeMax() const
{
  return this->rangeElem->GetValueDouble("max");
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
  return this->rangeElem->GetValueDouble("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRayCount() const
{
  return this->horzElem->GetValueUInt("samples");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetRangeCount() const
{
  return this->GetRayCount() *
        this->horzElem->GetValueDouble("resolution");
}

//////////////////////////////////////////////////
int GpuRaySensor::GetVerticalRayCount() const
{
  if (this->scanElem->HasElement("vertical"))
    return this->vertElem->GetValueUInt("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
int GpuRaySensor::GetVerticalRangeCount() const
{
  if (this->scanElem->HasElement("vertical"))
  {
    int rows =  (this->GetVerticalRayCount() *
          this->vertElem->GetValueDouble("resolution"));
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
    return this->vertElem->GetValueDouble("min_angle");
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
    return this->vertElem->GetValueDouble("max_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
void GpuRaySensor::SetVerticalAngleMax(double _angle)
{
  if (this->scanElem->HasElement("vertical"))
    this->vertElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void GpuRaySensor::GetRanges(std::vector<double> &/*_ranges*/) const
{
  /*boost::mutex::scoped_lock(this->mutex);
  _ranges.resize(this->laserMsg.ranges_size());
  memcpy(&_ranges[0], this->laserMsg.ranges().data(),
         sizeof(_ranges[0]) * this->laserMsg.ranges_size());*/
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRange(int _index)
{
  if (_index < 0 || _index > this->laserMsg.ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  boost::mutex::scoped_lock lock(this->mutex);
  return this->laserMsg.ranges(_index);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRetro(int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int GpuRaySensor::GetFiducial(int /*_index*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void GpuRaySensor::UpdateImpl(bool /*_force*/)
{
  if (this->laserCam)
  {
    this->laserCam->Render();
    this->laserCam->PostRender();
    this->lastMeasurementTime = this->world->GetSimTime();
  }
}
