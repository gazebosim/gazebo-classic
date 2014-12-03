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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("multicamera", MultiCameraSensor)

//////////////////////////////////////////////////
MultiCameraSensor::MultiCameraSensor()
    : Sensor(sensors::IMAGE)
{
  this->rendered = false;
  this->connections.push_back(
      event::Events::ConnectRender(
        boost::bind(&MultiCameraSensor::Render, this)));
}

//////////////////////////////////////////////////
MultiCameraSensor::~MultiCameraSensor()
{
}

//////////////////////////////////////////////////
std::string MultiCameraSensor::GetTopic() const
{
  std::string topic = Sensor::GetTopic();

  // Create a topic name if one has not been specified.
  if (topic.empty())
  {
    topic = "~/";
    topic += this->parentName + "/" + this->GetName() + "/images";
    boost::replace_all(topic, "::", "/");
  }

  return topic;
}

//////////////////////////////////////////////////
void MultiCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  // Create the publisher of image data.
  this->imagePub = this->node->Advertise<msgs::ImagesStamped>(
      this->GetTopic(), 50);
}

//////////////////////////////////////////////////
void MultiCameraSensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create MultiCameraSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->GetName();

  if (worldName.empty())
  {
    gzerr << "No world name\n";
    return;
  }

  this->scene = rendering::get_scene(worldName);

  if (!this->scene)
  {
    this->scene = rendering::create_scene(worldName, false, true);

    // This usually means rendering is not available
    if (!this->scene)
    {
      gzerr << "Unable to create MultiCameraSensor.\n";
      return;
    }
  }

  // Create and initialize all the cameras
  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
  while (cameraSdf)
  {
    rendering::CameraPtr camera = scene->CreateCamera(
          cameraSdf->Get<std::string>("name"), false);

    if (!camera)
    {
      gzthrow("Unable to create multicamera sensor[" +
              cameraSdf->Get<std::string>("name"));
      return;
    }

    camera->SetCaptureData(true);
    camera->Load(cameraSdf);

    // Do some sanity checks
    if (camera->GetImageWidth() == 0 || camera->GetImageHeight() == 0)
      gzthrow("Image has zero size");

    camera->Init();
    camera->CreateRenderTexture(camera->GetName() + "_RttTex");

    math::Pose cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<math::Pose>("pose") + cameraPose;
    camera->SetWorldPose(cameraPose);
    camera->AttachToVisual(this->parentId, true);

    // Handle noise model settings.
    if (cameraSdf->HasElement("noise"))
    {
      NoisePtr noise =
          NoiseFactory::NewNoiseModel(cameraSdf->GetElement("noise"),
          this->GetType());
      this->noises.push_back(noise);
      noise->SetCamera(camera);
    }
    else
    {
      this->noises.push_back(NoisePtr(new Noise(Noise::NONE)));
    }

    {
      boost::mutex::scoped_lock lock(this->cameraMutex);
      this->cameras.push_back(camera);
    }

    msgs::Image *image = this->msg.add_image();
    image->set_width(camera->GetImageWidth());
    image->set_height(camera->GetImageHeight());
    image->set_pixel_format(common::Image::ConvertPixelFormat(
          camera->GetImageFormat()));
    image->set_step(camera->GetImageWidth() * camera->GetImageDepth());

    cameraSdf = cameraSdf->GetNextElement("camera");
  }

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

//////////////////////////////////////////////////
void MultiCameraSensor::Fini()
{
  this->imagePub.reset();
  Sensor::Fini();

  boost::mutex::scoped_lock lock(this->cameraMutex);

  for (std::vector<rendering::CameraPtr>::iterator iter =
      this->cameras.begin(); iter != this->cameras.end(); ++iter)
  {
    (*iter)->GetScene()->RemoveCamera((*iter)->GetName());
  }
  this->cameras.clear();
  this->scene.reset();
}

//////////////////////////////////////////////////
rendering::CameraPtr MultiCameraSensor::GetCamera(unsigned int _index) const
{
  boost::mutex::scoped_lock lock(this->cameraMutex);

  if (_index < this->cameras.size())
    return this->cameras[_index];
  else
    gzthrow("camera index out of range. Valid range[0.." +
        boost::lexical_cast<std::string>(this->cameras.size()-1));
}

//////////////////////////////////////////////////
void MultiCameraSensor::Render()
{
  if (this->cameras.empty() || !this->IsActive() || !this->NeedsUpdate())
    return;

  // Update all the cameras
  for (std::vector<rendering::CameraPtr>::iterator iter = this->cameras.begin();
      iter != this->cameras.end(); ++iter)
  {
    (*iter)->Render();
  }

  this->rendered = true;
  this->lastMeasurementTime = this->scene->GetSimTime();
}

//////////////////////////////////////////////////
bool MultiCameraSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->cameraMutex);

  if (!this->rendered)
    return false;

  bool publish = this->imagePub->HasConnections();

  msgs::Set(this->msg.mutable_time(), this->lastMeasurementTime);

  int index = 0;
  for (std::vector<rendering::CameraPtr>::iterator iter = this->cameras.begin();
       iter != this->cameras.end(); ++iter, ++index)
  {
    (*iter)->PostRender();

    if (publish)
    {
      msgs::Image *image = this->msg.mutable_image(index);
      image->set_data((*iter)->GetImageData(0),
          image->width() * (*iter)->GetImageDepth() * image->height());
    }
  }

  if (publish)
    this->imagePub->Publish(this->msg);

  this->rendered = false;
  return true;
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::GetCameraCount() const
{
  boost::mutex::scoped_lock lock(this->cameraMutex);
  return this->cameras.size();
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::GetImageWidth(unsigned int _index) const
{
  return this->GetCamera(_index)->GetImageWidth();
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::GetImageHeight(unsigned int _index) const
{
  return this->GetCamera(_index)->GetImageHeight();
}

//////////////////////////////////////////////////
const unsigned char *MultiCameraSensor::GetImageData(unsigned int _index)
{
  return this->GetCamera(_index)->GetImageData(0);
}

//////////////////////////////////////////////////
bool MultiCameraSensor::SaveFrame(const std::vector<std::string> &_filenames)
{
  this->SetActive(true);

  boost::mutex::scoped_lock lock(this->cameraMutex);
  if (_filenames.size() != this->cameras.size())
  {
    gzerr << "Filename count[" << _filenames.size() << "] does not match "
          << "camera count[" << this->cameras.size() << "]\n";
    return false;
  }

  bool result = true;

  std::vector<rendering::CameraPtr>::iterator citer = this->cameras.begin();
  for (std::vector<std::string>::const_iterator fiter = _filenames.begin();
       fiter != _filenames.end(); ++fiter, ++citer)
  {
    result &= (*citer)->SaveFrame(*fiter);
  }

  return result;
}

//////////////////////////////////////////////////
bool MultiCameraSensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->imagePub && this->imagePub->HasConnections());
}
