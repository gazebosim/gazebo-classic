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

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Camera.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"

#include "gazebo/sensors/CameraSensorPrivate.hh"
#include "gazebo/sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
: Sensor(*new CameraSensorPrivate, sensors::IMAGE),
  dataPtr(std::static_pointer_cast<CameraSensorPrivate>(this->dPtr))
{
  this->dataPtr->rendered = false;
  this->dataPtr->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&CameraSensor::Render, this)));
}

//////////////////////////////////////////////////
CameraSensor::CameraSensor(CameraSensorPrivate &_dataPtr)
: Sensor(_dataPtr, sensors::IMAGE)
{
  this->dataPtr = std::static_pointer_cast<CameraSensorPrivate>(this->dPtr);

  this->dataPtr->rendered = false;
  this->dataPtr->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&CameraSensor::Render, this)));
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
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
  this->dataPtr->imagePub = this->dataPtr->node->Advertise<msgs::ImageStamped>(
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

  std::string worldName = this->dataPtr->world->GetName();

  if (!worldName.empty())
  {
    this->dataPtr->scene = rendering::get_scene(worldName);
    if (!this->dataPtr->scene)
    {
      this->dataPtr->scene = rendering::create_scene(worldName, false, true);

      // This usually means rendering is not available
      if (!this->dataPtr->scene)
      {
        gzerr << "Unable to create CameraSensor.\n";
        return;
      }
    }

    this->dataPtr->camera = this->dataPtr->scene->CreateCamera(
        this->dataPtr->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
      return;
    }
    this->dataPtr->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->dataPtr->sdf->GetElement("camera");
    this->dataPtr->camera->Load(cameraSdf);

    // Do some sanity checks
    if (this->dataPtr->camera->GetImageWidth() == 0 ||
        this->dataPtr->camera->GetImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->dataPtr->camera->Init();
    this->dataPtr->camera->CreateRenderTexture(this->Name() + "_RttTex");
    ignition::math::Pose3d cameraPose = this->dataPtr->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

    this->dataPtr->camera->SetWorldPose(cameraPose);
    this->dataPtr->camera->AttachToVisual(this->ParentId(), true);

    if (cameraSdf->HasElement("noise"))
    {
      this->dataPtr->noises[CAMERA_NOISE] =
        NoiseFactory::NewNoiseModel(cameraSdf->GetElement("noise"),
        this->Type());
      this->dataPtr->noises[CAMERA_NOISE]->SetCamera(this->dataPtr->camera);
    }
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
void CameraSensor::Fini()
{
  this->dataPtr->imagePub.reset();
  Sensor::Fini();

  if (this->dataPtr->camera)
  {
    this->dataPtr->scene->RemoveCamera(this->dataPtr->camera->GetName());
  }

  this->dataPtr->camera.reset();
  this->dataPtr->scene.reset();
}

//////////////////////////////////////////////////
void CameraSensor::Render()
{
  if (!this->dataPtr->camera || !this->IsActive() || !this->NeedsUpdate())
    return;

  // Update all the cameras
  this->dataPtr->camera->Render();

  this->dataPtr->rendered = true;
  this->dataPtr->lastMeasurementTime = this->dataPtr->scene->GetSimTime();
}

//////////////////////////////////////////////////
bool CameraSensor::UpdateImpl(const bool /*_force*/)
{
  if (!this->dataPtr->rendered)
    return false;

  this->dataPtr->camera->PostRender();

  if (this->dataPtr->imagePub && this->dataPtr->imagePub->HasConnections())
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->dataPtr->scene->GetSimTime());
    msg.mutable_image()->set_width(this->dataPtr->camera->GetImageWidth());
    msg.mutable_image()->set_height(this->dataPtr->camera->GetImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
          this->dataPtr->camera->GetImageFormat()));

    msg.mutable_image()->set_step(this->dataPtr->camera->GetImageWidth() *
        this->dataPtr->camera->GetImageDepth());
    msg.mutable_image()->set_data(this->dataPtr->camera->GetImageData(),
        msg.image().width() * this->dataPtr->camera->GetImageDepth() *
        msg.image().height());

    this->dataPtr->imagePub->Publish(msg);
  }

  this->dataPtr->rendered = false;
  return true;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::GetImageWidth() const
{
  return this->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageWidth() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->GetImageWidth();

  sdf::ElementPtr cameraSdf = this->dataPtr->sdf->GetElement("camera");
  sdf::ElementPtr elem = cameraSdf->GetElement("image");
  return elem->Get<unsigned int>("width");
}

//////////////////////////////////////////////////
unsigned int CameraSensor::GetImageHeight() const
{
  return this->ImageHeight();
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageHeight() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->GetImageHeight();

  sdf::ElementPtr cameraSdf = this->dataPtr->sdf->GetElement("camera");
  sdf::ElementPtr elem = cameraSdf->GetElement("image");
  return elem->Get<unsigned int>("height");
}

//////////////////////////////////////////////////
const unsigned char *CameraSensor::GetImageData()
{
  return this->ImageData();
}

//////////////////////////////////////////////////
const unsigned char *CameraSensor::ImageData() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->GetImageData(0);
  else
    return NULL;
}

//////////////////////////////////////////////////
bool CameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);

  if (this->dataPtr->camera)
    return this->dataPtr->camera->SaveFrame(_filename);
  else
    return false;
}

//////////////////////////////////////////////////
bool CameraSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->imagePub && this->dataPtr->imagePub->HasConnections());
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::GetCamera() const
{
  return this->Camera();
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::Camera() const
{
  return this->dataPtr->camera;
}
