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

#include <functional>

#include "gazebo/physics/World.hh"

#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/DepthCameraSensorPrivate.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
    : CameraSensor(),
      dataPtr(new DepthCameraSensorPrivate)
{
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
void DepthCameraSensor::Load(const std::string &_worldName)
{
  CameraSensor::Load(_worldName);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create DepthCameraSensor. Rendering is disabled."
        << std::endl;
    return;
  }

  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    this->dataPtr->depthCamera = this->scene->CreateDepthCamera(
        this->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->depthCamera)
    {
      gzerr << "Unable to create depth camera sensor" << std::endl;
      return;
    }
    this->dataPtr->depthCamera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    this->dataPtr->depthCamera->Load(cameraSdf);

    // Do some sanity checks
    if (this->dataPtr->depthCamera->ImageWidth() == 0u ||
        this->dataPtr->depthCamera->ImageHeight() == 0u)
    {
      gzerr << "image has zero size" << std::endl;
    }

    this->dataPtr->depthCamera->Init();
    this->dataPtr->depthCamera->CreateRenderTexture(
        this->Name() + "_RttTex_Image");
    this->dataPtr->depthCamera->CreateDepthTexture(
        this->Name() + "_RttTex_Depth");
    ignition::math::Pose3d cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

    this->dataPtr->depthCamera->SetWorldPose(cameraPose);
    this->dataPtr->depthCamera->AttachToVisual(this->parentId, true, 0, 0);

    this->camera = boost::dynamic_pointer_cast<rendering::Camera>(
        this->dataPtr->depthCamera);

    GZ_ASSERT(this->camera, "Unable to cast depth camera to camera");
  }
  else
  {
    gzerr << "No world name" << std::endl;
  }

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

//////////////////////////////////////////////////
bool DepthCameraSensor::UpdateImpl(const bool /*_force*/)
{
  if (!this->Rendered())
    return false;

  this->camera->PostRender();

  if (this->imagePub && this->imagePub->HasConnections())
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->scene->SimTime());
    msg.mutable_image()->set_width(this->camera->ImageWidth());
    msg.mutable_image()->set_height(this->camera->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);


    msg.mutable_image()->set_step(this->camera->ImageWidth() *
        this->camera->ImageDepth());

    unsigned int depthSamples = msg.image().width() * msg.image().height();
    float f;
    // cppchecker recommends using sizeof(varname)
    unsigned int depthBufferSize = depthSamples * sizeof(f);

    if (!this->dataPtr->depthBuffer)
      this->dataPtr->depthBuffer = new float[depthSamples];

    memcpy(this->dataPtr->depthBuffer, this->dataPtr->depthCamera->DepthData(),
        depthBufferSize);

    for (unsigned int i = 0; i < depthSamples; ++i)
    {
      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (this->dataPtr->depthBuffer[i] >= this->camera->FarClip())
      {
        this->dataPtr->depthBuffer[i] = IGN_DBL_INF;
      }
      else if (this->dataPtr->depthBuffer[i] <= this->camera->NearClip())
      {
        this->dataPtr->depthBuffer[i] = -IGN_DBL_INF;
      }
    }
    msg.mutable_image()->set_data(this->dataPtr->depthBuffer, depthBufferSize);
    this->imagePub->Publish(msg);
  }

  this->SetRendered(false);
  return true;
}

//////////////////////////////////////////////////
const float *DepthCameraSensor::DepthData() const
{
  return this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
rendering::DepthCameraPtr DepthCameraSensor::DepthCamera() const
{
  return this->dataPtr->depthCamera;
}
