/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
    : Sensor(sensors::IMAGE)
{
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
std::string CameraSensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/image";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void CameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->imagePub = this->node->Advertise<msgs::ImageStamped>(
      this->GetTopic(), 50);
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

    this->camera = this->scene->CreateCamera(
        this->sdf->Get<std::string>("name"), false);

    if (!this->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
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
    this->camera->CreateRenderTexture(this->GetName() + "_RttTex");
    math::Pose cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<math::Pose>("pose") + cameraPose;

    this->camera->SetWorldPose(cameraPose);
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
void CameraSensor::Fini()
{
  this->imagePub.reset();
  Sensor::Fini();

  if (this->camera)
  {
    this->scene->RemoveCamera(this->camera->GetName());
  }

  this->camera.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
void CameraSensor::UpdateImpl(bool /*_force*/)
{
  if (this->camera)
  {
    this->camera->Render();
    this->camera->PostRender();
    this->lastMeasurementTime = this->scene->GetSimTime();

    if (this->imagePub->HasConnections())
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), this->scene->GetSimTime());
      msg.mutable_image()->set_width(this->camera->GetImageWidth());
      msg.mutable_image()->set_height(this->camera->GetImageHeight());
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
            this->camera->GetImageFormat()));

      msg.mutable_image()->set_step(this->camera->GetImageWidth() *
          this->camera->GetImageDepth());
      msg.mutable_image()->set_data(this->camera->GetImageData(),
          msg.image().width() * this->camera->GetImageDepth() *
          msg.image().height());

      if (this->imagePub && this->imagePub->HasConnections())
        this->imagePub->Publish(msg);
    }
  }
}

//////////////////////////////////////////////////
unsigned int CameraSensor::GetImageWidth() const
{
  if (this->camera)
    return this->camera->GetImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::GetImageHeight() const
{
  if (this->camera)
    return this->camera->GetImageHeight();
  return 0;
}

//////////////////////////////////////////////////
const unsigned char *CameraSensor::GetImageData()
{
  return this->camera->GetImageData(0);
}

//////////////////////////////////////////////////
bool CameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);
  return this->camera->SaveFrame(_filename);
}

//////////////////////////////////////////////////
bool CameraSensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->imagePub && this->imagePub->HasConnections());
}
