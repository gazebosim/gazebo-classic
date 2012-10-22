/*
 * Copyright 2011 Nate Koenig
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

#include "common/Events.hh"
#include "common/Exception.hh"

#include "transport/transport.hh"
#include "msgs/msgs.hh"

#include "physics/World.hh"

#include "rendering/RenderEngine.hh"
#include "rendering/Camera.hh"
#include "rendering/Scene.hh"
#include "rendering/Rendering.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
    : Sensor()
{
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
}

//////////////////////////////////////////////////
void CameraSensor::SetParent(const std::string &_name)
{
  Sensor::SetParent(_name);
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
  this->imagePub = this->node->Advertise<msgs::ImageStamped>(this->GetTopic());
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
      this->scene = rendering::create_scene(worldName, false);

      // This usually means rendering is not available
      if (!this->scene)
      {
        gzerr << "Unable to create CameraSensor.\n";
        return;
      }
    }

    this->camera = this->scene->CreateCamera(
        this->sdf->GetValueString("name"), false);

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
    this->camera->SetWorldPose(this->pose);
    this->camera->AttachToVisual(this->parentName, true);
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
}

//////////////////////////////////////////////////
void CameraSensor::Fini()
{
  Sensor::Fini();
  if (this->camera)
    this->camera->Fini();
  this->camera.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
void CameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////
void CameraSensor::UpdateImpl(bool /*_force*/)
{
  if (this->camera)
  {
    this->camera->Render();
    this->camera->PostRender();
    this->lastUpdateTime = this->world->GetSimTime();

    if (this->imagePub->HasConnections())
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), this->world->GetSimTime());
      msg.mutable_image()->set_width(this->camera->GetImageWidth());
      msg.mutable_image()->set_height(this->camera->GetImageHeight());
      // msg.mutable_image()->set_pixel_format(this->camera->GetImageFormat());
      msg.mutable_image()->set_step(this->camera->GetImageWidth() * 3);
      msg.mutable_image()->set_data(this->camera->GetImageData(),
          msg.image().width() * 3 * msg.image().height());
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
