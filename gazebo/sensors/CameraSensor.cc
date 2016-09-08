/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <functional>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"

#include "gazebo/sensors/CameraSensorPrivate.hh"
#include "gazebo/sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
: Sensor(sensors::IMAGE),
  dataPtr(new CameraSensorPrivate)
{
  this->connections.push_back(
      event::Events::ConnectRender(
        std::bind(&CameraSensor::Render, this)));
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
void CameraSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
std::string CameraSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/image";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void CameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->imagePub = this->node->Advertise<msgs::ImageStamped>(
      this->Topic(), 50);
}

//////////////////////////////////////////////////
void CameraSensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create CameraSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);
    if (!this->scene)
    {
      this->scene = rendering::create_scene(worldName, false, true);

      // This usually means rendering is not available
      if (!this->scene)
      {
        gzerr << "Unable to create CameraSensor.\n";
        return;
      }
    }

    std::string scopedName = this->parentName + "::" + this->Name();
    this->camera = this->scene->CreateCamera(scopedName, false);

    if (!this->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
      return;
    }
    this->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    this->camera->Load(cameraSdf);

    // Do some sanity checks
    if (this->camera->ImageWidth() == 0 ||
        this->camera->ImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->camera->Init();
    this->camera->CreateRenderTexture(scopedName + "_RttTex");
    ignition::math::Pose3d cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

    this->camera->SetWorldPose(cameraPose);
    this->camera->AttachToVisual(this->ParentId(), true, 0, 0);

    if (cameraSdf->HasElement("noise"))
    {
      this->noises[CAMERA_NOISE] =
        NoiseFactory::NewNoiseModel(cameraSdf->GetElement("noise"),
        this->Type());
      this->noises[CAMERA_NOISE]->SetCamera(this->camera);
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
void CameraSensor::Fini()
{
  this->imagePub.reset();

  if (this->camera)
  {
    this->scene->RemoveCamera(this->camera->Name());
  }

  this->camera.reset();

  Sensor::Fini();
}

//////////////////////////////////////////////////
void CameraSensor::Render()
{
  if (!this->camera || !this->IsActive() || !this->NeedsUpdate())
    return;

  // Update all the cameras
  this->camera->Render();

  this->dataPtr->rendered = true;
  this->lastMeasurementTime = this->scene->SimTime();
}

//////////////////////////////////////////////////
bool CameraSensor::UpdateImpl(const bool /*_force*/)
{
  if (!this->dataPtr->rendered)
    return false;

  this->camera->PostRender();

  if (this->imagePub && this->imagePub->HasConnections())
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->scene->SimTime());
    msg.mutable_image()->set_width(this->camera->ImageWidth());
    msg.mutable_image()->set_height(this->camera->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
          this->camera->ImageFormat()));

    msg.mutable_image()->set_step(this->camera->ImageWidth() *
        this->camera->ImageDepth());
    msg.mutable_image()->set_data(this->camera->ImageData(),
        msg.image().width() * this->camera->ImageDepth() *
        msg.image().height());

    this->imagePub->Publish(msg);
  }

  this->dataPtr->rendered = false;
  return true;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageWidth() const
{
  if (this->camera)
    return this->camera->ImageWidth();

  if (this->sdf && this->sdf->HasElement("camera"))
  {
    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    sdf::ElementPtr elem = cameraSdf->GetElement("image");
    return elem->Get<unsigned int>("width");
  }

  gzwarn << "Can't get image width." << std::endl;
  return 0;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageHeight() const
{
  if (this->camera)
    return this->camera->ImageHeight();

  if (this->sdf && this->sdf->HasElement("camera"))
  {
    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    sdf::ElementPtr elem = cameraSdf->GetElement("image");
    return elem->Get<unsigned int>("height");
  }

  gzwarn << "Can't get image height." << std::endl;
  return 0;
}

//////////////////////////////////////////////////
const unsigned char *CameraSensor::ImageData() const
{
  if (this->camera)
    return this->camera->ImageData(0);
  else
    return nullptr;
}

//////////////////////////////////////////////////
bool CameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);

  if (this->camera)
    return this->camera->SaveFrame(_filename);
  else
    return false;
}

//////////////////////////////////////////////////
bool CameraSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->imagePub && this->imagePub->HasConnections());
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::Camera() const
{
  return this->camera;
}

//////////////////////////////////////////////////
bool CameraSensor::Rendered() const
{
  return this->dataPtr->rendered;
}

//////////////////////////////////////////////////
void CameraSensor::SetRendered(const bool _value)
{
  this->dataPtr->rendered = _value;
}

