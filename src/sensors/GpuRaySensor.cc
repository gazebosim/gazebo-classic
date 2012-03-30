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
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
*/

#include "physics/World.hh"

#include "common/Exception.hh"
#include "common/Events.hh"

#include "transport/transport.h"

#include "rendering/Scene.hh"
#include "rendering/Rendering.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/GpuRaySensor.hh"
#include "rendering/GpuLaser.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gpu_ray", GpuRaySensor)

//////////////////////////////////////////////////
GpuRaySensor::GpuRaySensor()
    : Sensor()
{
  this->mutex = new boost::mutex();
  this->active = false;
  this->node = transport::NodePtr(new transport::Node());
}

//////////////////////////////////////////////////
GpuRaySensor::~GpuRaySensor()
{
  delete this->mutex;
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
  this->poseSub = this->node->Subscribe("~/pose",
                                        &GpuRaySensor::OnPose, this);
  
  this->rayElem = this->sdf->GetElement("ray");
  this->scanElem = this->rayElem->GetElement("scan");
  this->horzElem = this->scanElem->GetElement("horizontal");
  this->rangeElem = this->rayElem->GetElement("range");

  this->isHorizontal = (this->scanElem->GetValueString("sensor_plane") == "horizontal") ? true : false;

  if (this->scanElem->HasElement("vertical"))
    this->vertElem = this->scanElem->GetElement("vertical");

  width_1st = this->GetRayCount();
  height_1st = this->GetVerticalRayCount();

  if (width_1st == 0 || height_1st == 0)
  {
    gzthrow("GpuRaySensor: Image has 0 size!");
  }
  
  width_2nd = this->GetRangeCount();
  height_2nd = this->GetVerticalRangeCount();

  if (height_1st == 1)
    height_2nd = 1;

  ratio_2nd = width_2nd / height_2nd;

  near = this->GetRangeMin();
  far = this->GetRangeMax();

  hfov = (this->GetAngleMax() - this->GetAngleMin()).GetAsRadian();
  vfov = (this->GetVerticalAngleMax() - this->GetVerticalAngleMin()).GetAsRadian();

  hang = (this->GetAngleMax() + this->GetAngleMin()).GetAsRadian() / 2.0;
  vang = (this->GetVerticalAngleMax() + this->GetVerticalAngleMin()).GetAsRadian() / 2.0;

  if (hfov > 2 * M_PI)
    hfov = 2*M_PI;

  this->cameraCount = 1;

  if (hfov > 2.8)
  {
    if (hfov > 5.6)
      this->cameraCount = 3;
    else
      this->cameraCount = 2;
  }

  hfov /= this->cameraCount;
  width_1st /= this->cameraCount;

  if (vfov > M_PI / 2)
  {
    gzwarn << "Vertical FOV for block GPU laser model is capped at 90 degrees.\n";
    vfov = M_PI / 2;
    this->SetVerticalAngleMin(vang - (vfov / 2)); 
    this->SetVerticalAngleMax(vang + (vfov / 2)); 
  }

  if (height_1st > 1 && this->IsHorizontal() && this->cameraCount > 1)
  {
    gzwarn << "For horizontal FOV greater than 160 degrees consider using vertical sensor model for block lasers.\n";
    gzwarn << "Using horizontal model will output incorrect ranges.\n";
  }

  if ((width_1st * height_1st) < (width_2nd * height_2nd))
  {
    width_1st = std::max(width_1st, width_2nd);
    height_1st = std::max(height_1st, height_2nd);
  }

  //gzwarn <<"Sensor plane: " << ((this->isHorizontal) ? "horizontal":"vertical") <<"\n";
  //gzwarn << "Camera count: " << this->cameraCount << 
  //          " Camera width: " << width_1st << 
  //          " height: " << height_1st << "\n";

  if (this->isHorizontal)
  {
    if (height_1st > 1)
    {
      chfov = 2 * atan(tan(hfov/2) / cos(vfov/2));
      cvfov = vfov;
      ratio_1st = tan(chfov/2.0) / tan(vfov/2.0);
      
      //gzwarn << "Corrected HFOV: " << chfov << " (HFOV: " << hfov << ")\n";
      
      if ((width_1st / ratio_1st) > height_1st)
        height_1st = width_1st / ratio_1st;
      else
        width_1st = height_1st * ratio_1st;
    }
    else
    {
      chfov = hfov;
      cvfov = vfov;
    }
  }
  else
  {
    if (width_1st > 1)
    {
      chfov = hfov;
      cvfov = 2 * atan(tan(vfov/2) / cos(hfov/2));
      ratio_1st = tan(hfov/2.0) / tan(cvfov/2.0);
    
      //gzwarn << "Corrected VFOV: " << cvfov << " (VFOV: " << vfov << ")\n";
      
      if ((width_1st / ratio_1st) > height_1st)
        height_1st = width_1st / ratio_1st;
      else
        width_1st = height_1st * ratio_1st;
    }
    else
    {
      chfov = hfov;
      cvfov = vfov;
    }
  }
    
  //gzwarn << "CHFOV: " << chfov << " CVFOV: " << cvfov << "\n";
  //gzwarn << "Final texture size: " << width_1st << " x " << height_1st << "\n";
  //gzwarn << "Range configuration: " << width_2nd << " x " << height_2nd << "\n";

  this->cameraElem.reset(new sdf::Element);
  sdf::initFile("sdf/camera.sdf", this->cameraElem);

  sdf::ElementPtr ptr = this->cameraElem->GetOrCreateElement("horizontal_fov");
  ptr->GetAttribute("angle")->Set(chfov);

  ptr = this->cameraElem->GetOrCreateElement("image");
  ptr->GetAttribute("width")->Set(width_1st);
  ptr->GetAttribute("height")->Set(height_1st);
  ptr->GetAttribute("format")->Set("R8G8B8");
  
  ptr = this->cameraElem->GetOrCreateElement("clip");
  ptr->GetAttribute("near")->Set(near);
  ptr->GetAttribute("far")->Set(far);
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
    this->laserCam->SetRangeCount(width_2nd, height_2nd);
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
unsigned int GpuRaySensor::GetCameraCount()
{
  return this->cameraCount;
}
  
//////////////////////////////////////////////////
bool GpuRaySensor::IsHorizontal()
{
  return this->isHorizontal;
}
  
//////////////////////////////////////////////////
double GpuRaySensor::GetHAngle()
{
  return this->hang;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVAngle()
{
  return this->vang;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetHFOV()
{
  return this->hfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCHFOV()
{
  return this->chfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetVFOV()
{
  return this->vfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::GetCVFOV()
{
  return this->cvfov;
}

//////////////////////////////////////////////////
double GpuRaySensor::Get1stRatio()
{
  return this->ratio_1st;
}

//////////////////////////////////////////////////
double GpuRaySensor::Get2ndRatio()
{
  return this->ratio_2nd;
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMin() const
{
  return this->horzElem->GetValueDouble("min_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMin(double angle) 
{
  this->horzElem->GetAttribute("min_angle")->Set(angle);
}

//////////////////////////////////////////////////
math::Angle GpuRaySensor::GetAngleMax() const
{
  return this->horzElem->GetValueDouble("max_angle");
}

//////////////////////////////////////////////////
void GpuRaySensor::SetAngleMax(double angle)
{
  this->horzElem->GetAttribute("max_angle")->Set(angle);
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
  return (this->GetAngleMax() - this->GetAngleMin()).GetAsRadian() /
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
void GpuRaySensor::SetVerticalAngleMin(double angle)
{
  if (this->scanElem->HasElement("vertical"))
    this->vertElem->GetAttribute("min_angle")->Set(angle);
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
void GpuRaySensor::SetVerticalAngleMax(double angle)
{
  if (this->scanElem->HasElement("vertical"))
    this->vertElem->GetAttribute("max_angle")->Set(angle);
}

//////////////////////////////////////////////////
void GpuRaySensor::GetRanges(std::vector<double> &_ranges)
{
  /*boost::mutex::scoped_lock(*this->mutex);
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

  boost::mutex::scoped_lock(*this->mutex);
  return this->laserMsg.ranges(_index);
}

//////////////////////////////////////////////////
double GpuRaySensor::GetRetro(int index)
{
  //boost::mutex::scoped_lock(*this->mutex);
  //return this->laserShape->GetRetro(index);
}

//////////////////////////////////////////////////
int GpuRaySensor::GetFiducial(int index)
{
  //boost::mutex::scoped_lock(*this->mutex);
  //return this->laserShape->GetFiducial(index);
}

//////////////////////////////////////////////////
void GpuRaySensor::UpdateImpl(bool /*_force*/)
{
  //this->mutex->lock();

  // Sensor::Update(force);
  if (this->laserCam)\
  {
    this->laserCam->Render();
    this->laserCam->PostRender();
    this->lastUpdateTime = this->world->GetSimTime();
  }
  
  //this->mutex->unlock();

  //if (this->scanPub)
  //  this->scanPub->Publish(this->laserMsg);
}

void GpuRaySensor::OnPose(ConstPosePtr &/*_msg*/)
{
}


