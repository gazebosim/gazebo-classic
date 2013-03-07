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
/* Desc: A camera sensor using OpenGL
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <sstream>

#include "gazebo/physics/World.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Rendering.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("depth", DepthCameraSensor)

//////////////////////////////////////////////////
DepthCameraSensor::DepthCameraSensor()
    : Sensor(sensors::IMAGE)
{
}

//////////////////////////////////////////////////
DepthCameraSensor::~DepthCameraSensor()
{
}

//////////////////////////////////////////////////
void DepthCameraSensor::SetParent(const std::string &_name)
{
  Sensor::SetParent(_name);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Load(const std::string &_worldName,
                                   sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  std::string topicName = "~/" + this->parentName + "/" + this->GetName();
  boost::replace_all(topicName, "::", "/");

  std::string imageTopicName = topicName + "/image";
  std::string depthTopicName = topicName + "/depth";

  this->imagePub = this->node->Advertise<msgs::ImageStamped>(imageTopicName);
  this->depthPub = this->node->Advertise<msgs::ImageStamped>(depthTopicName);
}

//////////////////////////////////////////////////
void DepthCameraSensor::Init()
{
  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false);

    this->camera = this->scene->CreateDepthCamera(
        this->sdf->GetValueString("name"), false);

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
    this->camera->AttachToVisual(this->parentName, true);
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
}

//////////////////////////////////////////////////
void DepthCameraSensor::Fini()
{
  Sensor::Fini();
  this->camera->Fini();
  this->camera.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
void DepthCameraSensor::SetActive(bool value)
{
  Sensor::SetActive(value);
}

//////////////////////////////////////////////////
void DepthCameraSensor::UpdateImpl(bool /*_force*/)
{
  // Sensor::Update(force);
  if (this->camera)
  {
    this->camera->Render();
    this->camera->PostRender();
    this->lastMeasurementTime = this->world->GetSimTime();

    if (this->imagePub->HasConnections())
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), this->world->GetSimTime());
      msg.mutable_image()->set_width(this->camera->GetImageWidth());
      msg.mutable_image()->set_height(this->camera->GetImageHeight());
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
            this->camera->GetImageFormat()));

      msg.mutable_image()->set_step(this->camera->GetImageWidth() *
          this->camera->GetImageDepth());
      msg.mutable_image()->set_data(this->camera->GetImageData(),
          msg.image().width() * this->camera->GetImageDepth() *
          msg.image().height());
      this->imagePub->Publish(msg);
    }

    if (this->depthPub->HasConnections())
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), this->world->GetSimTime());
      msg.mutable_image()->set_width(this->camera->GetImageWidth());
      msg.mutable_image()->set_height(this->camera->GetImageHeight());
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
            this->camera->GetImageFormat()));

      msg.mutable_image()->set_step(this->camera->GetImageWidth() *
          this->camera->GetImageDepth());
      msg.mutable_image()->set_data(this->camera->GetImageData(),
          msg.image().width() * this->camera->GetImageDepth() *
          msg.image().height());
      this->depthPub->Publish(msg);
    }
  }
}

//////////////////////////////////////////////////
unsigned int DepthCameraSensor::GetImageWidth() const
{
  if (this->camera)
    return this->camera->GetImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int DepthCameraSensor::GetImageHeight() const
{
  if (this->camera)
    return this->camera->GetImageHeight();
  return 0;
}

//////////////////////////////////////////////////
const unsigned char *DepthCameraSensor::GetImageData()
{
  return this->camera->GetImageData(0);
}

//////////////////////////////////////////////////
bool DepthCameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);
  return this->camera->SaveFrame(_filename);
}

//////////////////////////////////////////////////
bool DepthCameraSensor::IsActive()
{
  return Sensor::IsActive() || this->imagePub->HasConnections();
}
