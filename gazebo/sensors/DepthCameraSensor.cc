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
/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

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
#include "gazebo/sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
    : Sensor(sensors::IMAGE)
{
  this->rendered = false;
  this->connections.push_back(
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

  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    this->camera = this->scene->CreateDepthCamera(
        this->sdf->Get<std::string>("name"), false);

    if (!this->camera)
    {
      gzerr << "Unable to create depth camera sensor\n";
      return;
    }
    this->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    this->camera->Load(cameraSdf);

    // Do some sanity checks
    if (this->camera->GetImageWidth() == 0 ||
        this->camera->GetImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->camera->Init();
    this->camera->CreateRenderTexture(this->GetName() + "_RttTex_Image");
    this->camera->CreateDepthTexture(this->GetName() + "_RttTex_Depth");
    this->camera->SetWorldPose(this->pose);
    this->camera->AttachToVisual(this->parentId, true);
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
void DepthCameraSensor::Fini()
{
  Sensor::Fini();
  this->scene->RemoveCamera(this->camera->GetName());
  this->camera.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
void DepthCameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Render()
{
  if (!this->camera || !this->IsActive() || !this->NeedsUpdate())
    return;

  this->camera->Render();

  this->rendered = true;
  this->lastMeasurementTime = this->scene->GetSimTime();
}

//////////////////////////////////////////////////
bool DepthCameraSensor::UpdateImpl(bool /*_force*/)
{
  // Sensor::Update(force);
  if (!this->rendered)
    return false;

  this->camera->PostRender();

  this->rendered = false;
  return true;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);
  return this->camera->SaveFrame(_filename);
}
