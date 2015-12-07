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

#include <boost/bind.hpp>
#include <sstream>

#include "gazebo/physics/World.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/DepthCameraSensorPrivate.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
: Sensor(*new DepthCameraSensorPrivate, sensors::IMAGE),
  dataPtr(std::static_pointer_cast<DepthCameraSensorPrivate>(this->sensorDPtr))
{
  this->dataPtr->rendered = false;
  this->dataPtr->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&DepthCameraSensor::Render, this)));
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
}

//////////////////////////////////////////////////
void DepthCameraSensor::Load(const std::string &_worldName,
                                   sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create DepthCameraSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->dataPtr->world->GetName();

  if (!worldName.empty())
  {
    this->dataPtr->scene = rendering::get_scene(worldName);

    if (!this->dataPtr->scene)
      this->dataPtr->scene = rendering::create_scene(worldName, false, true);

    this->dataPtr->camera = this->dataPtr->scene->CreateDepthCamera(
        this->dataPtr->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->camera)
    {
      gzerr << "Unable to create depth camera sensor\n";
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
    this->dataPtr->camera->CreateRenderTexture(this->Name() + "_RttTex_Image");
    this->dataPtr->camera->CreateDepthTexture(this->Name() + "_RttTex_Depth");
    this->dataPtr->camera->SetWorldPose(this->Pose());
    this->dataPtr->camera->AttachToVisual(this->ParentId(), true);
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
void DepthCameraSensor::Fini()
{
  Sensor::Fini();
  this->dataPtr->scene->RemoveCamera(this->dataPtr->camera->GetName());
  this->dataPtr->camera.reset();
  this->dataPtr->scene.reset();
}

//////////////////////////////////////////////////
void DepthCameraSensor::SetActive(const bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Render()
{
  if (!this->dataPtr->camera || !this->IsActive() || !this->NeedsUpdate())
    return;

  this->dataPtr->camera->Render();

  this->dataPtr->rendered = true;
  this->dataPtr->lastMeasurementTime = this->dataPtr->scene->GetSimTime();
}

//////////////////////////////////////////////////
bool DepthCameraSensor::UpdateImpl(const bool /*_force*/)
{
  // Sensor::Update(force);
  if (!this->dataPtr->rendered)
    return false;

  this->dataPtr->camera->PostRender();

  this->dataPtr->rendered = false;
  return true;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);
  return this->dataPtr->camera->SaveFrame(_filename);
}

//////////////////////////////////////////////////
rendering::DepthCameraPtr DepthCameraSensor::GetDepthCamera() const
{
  return this->DepthCamera();
}

//////////////////////////////////////////////////
rendering::DepthCameraPtr DepthCameraSensor::DepthCamera() const
{
  return this->dataPtr->camera;
}
