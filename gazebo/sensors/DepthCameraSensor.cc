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
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
    : CameraSensor()
{
  this->depthBuffer = NULL;
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
  if (this->depthBuffer)
    delete [] this->depthBuffer;
}

//////////////////////////////////////////////////
std::string DepthCameraSensor::GetPointCloudTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/points";
  boost::replace_all(topicName, "::", "/");

  return topicName;
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
    gzerr << "Unable to create DepthCameraSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    rendering::DepthCameraPtr depthCamera = this->scene->CreateDepthCamera(
        this->sdf->Get<std::string>("name"), false);

    if (!depthCamera)
    {
      gzerr << "Unable to create depth camera sensor\n";
      return;
    }
    depthCamera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    depthCamera->Load(cameraSdf);

    // Do some sanity checks
    if (depthCamera->GetImageWidth() == 0 ||
        depthCamera->GetImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    depthCamera->Init();
    depthCamera->CreateRenderTexture(this->GetName() + "_RttTex_Image");
    depthCamera->CreateDepthTexture(this->GetName() + "_RttTex_Depth");
    ignition::math::Pose3d cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

    depthCamera->SetWorldPose(cameraPose);
    depthCamera->AttachToVisual(this->parentId, true);

    this->camera = boost::dynamic_pointer_cast<rendering::Camera>(depthCamera);

    GZ_ASSERT(this->camera, "Unable to cast depth camera to camera");
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
rendering::DepthCameraPtr DepthCameraSensor::GetDepthCamera() const
{
  return boost::dynamic_pointer_cast<rendering::DepthCamera>(this->camera);
}

//////////////////////////////////////////////////
const float *DepthCameraSensor::GetDepthData() const
{
  return this->depthBuffer;
}

//////////////////////////////////////////////////
bool DepthCameraSensor::UpdateImpl(bool /*_force*/)
{
  if (!this->rendered)
    return false;

  this->camera->PostRender();

  if (this->imagePub && this->imagePub->HasConnections())
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->scene->GetSimTime());
    msg.mutable_image()->set_width(this->camera->GetImageWidth());
    msg.mutable_image()->set_height(this->camera->GetImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);


    msg.mutable_image()->set_step(this->camera->GetImageWidth() *
        this->camera->GetImageDepth());

    rendering::DepthCameraPtr depthCamera =
        boost::dynamic_pointer_cast<rendering::DepthCamera>(this->camera);

    unsigned int depthSamples = msg.image().width() * msg.image().height();
    float f;
    // cppchecker recommends using sizeof(varname)
    unsigned int depthBufferSize = depthSamples * sizeof(f);

    if (!this->depthBuffer)
      this->depthBuffer = new float[depthSamples];

    memcpy(this->depthBuffer, depthCamera->GetDepthData(), depthBufferSize);

    for (unsigned int i = 0; i < depthSamples; ++i)
    {
      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (this->depthBuffer[i] >= this->camera->GetFarClip())
      {
        this->depthBuffer[i] = IGN_DBL_INF;
      }
      else if (this->depthBuffer[i] <= this->camera->GetNearClip())
      {
        this->depthBuffer[i] = -IGN_DBL_INF;
      }
    }
    msg.mutable_image()->set_data(this->depthBuffer, depthBufferSize);
    this->imagePub->Publish(msg);
  }

  this->rendered = false;
  return true;
}
