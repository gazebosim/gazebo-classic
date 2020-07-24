/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>
#include <functional>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/EnumIface.hh"
#include "gazebo/common/Image.hh"

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"

#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/MultiCameraSensorPrivate.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("multicamera", MultiCameraSensor)

//////////////////////////////////////////////////
MultiCameraSensor::MultiCameraSensor()
: Sensor(sensors::IMAGE),
  dataPtr(new MultiCameraSensorPrivate)
{
  this->dataPtr->rendered = false;
  this->connections.push_back(
      event::Events::ConnectRender(
        std::bind(&MultiCameraSensor::Render, this)));
}

//////////////////////////////////////////////////
MultiCameraSensor::~MultiCameraSensor()
{
}

//////////////////////////////////////////////////
std::string MultiCameraSensor::Topic() const
{
  std::string topic = Sensor::Topic();

  // Create a topic name if one has not been specified.
  if (topic.empty())
  {
    topic = "~/";
    topic += this->ParentName() + "/" + this->Name() + "/images";
    boost::replace_all(topic, "::", "/");
  }

  return topic;
}

//////////////////////////////////////////////////
void MultiCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  // useStrictRate is set in Sensor::Load()
  if (this->useStrictRate)
  {
    this->connections.push_back(
        event::Events::ConnectPreRenderEnded(
          boost::bind(&MultiCameraSensor::PrerenderEnded, this)));
  }

  // Create the publisher of image data.
  this->dataPtr->imagePub =
    this->node->Advertise<msgs::ImagesStamped>(this->Topic(), 50);
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

  std::string worldName = this->world->Name();

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

  // Each camera has its own noise pointer
  common::EnumIterator<SensorNoiseType> noiseIndex = SENSOR_NOISE_TYPE_BEGIN;

  // Create and initialize all the cameras
  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
  while (cameraSdf)
  {
    rendering::CameraPtr camera = this->scene->CreateCamera(
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
    if (camera->ImageWidth() == 0 || camera->ImageHeight() == 0)
      gzthrow("Image has zero size");

    camera->Init();
    camera->CreateRenderTexture(camera->Name() + "_RttTex");

    ignition::math::Pose3d cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;
    camera->SetWorldPose(cameraPose);
    camera->AttachToVisual(this->parentId, true, 0, 0);

    if (cameraSdf->HasElement("noise"))
    {
      // Create a noise model and attach the camera
      this->noises[*noiseIndex] = NoiseFactory::NewNoiseModel(
        cameraSdf->GetElement("noise"), this->Type());
      this->noises[*noiseIndex]->SetCamera(camera);
    }
    else
    {
      this->noises[*noiseIndex].reset(
          new sensors::Noise(sensors::Noise::NONE));
    }

    // Increment the noise index -- one for each camera in the setup
    ++noiseIndex;

    {
      std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);
      this->dataPtr->cameras.push_back(camera);
    }

    msgs::Image *image = this->dataPtr->msg.add_image();
    image->set_width(camera->ImageWidth());
    image->set_height(camera->ImageHeight());
    image->set_pixel_format(common::Image::ConvertPixelFormat(
          camera->ImageFormat()));
    image->set_step(camera->ImageWidth() * camera->ImageDepth());

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
  this->dataPtr->imagePub.reset();

  std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);

  for (std::vector<rendering::CameraPtr>::iterator iter =
      this->dataPtr->cameras.begin();
      iter != this->dataPtr->cameras.end(); ++iter)
  {
    this->scene->RemoveCamera((*iter)->Name());
  }
  this->dataPtr->cameras.clear();
  this->scene.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
rendering::CameraPtr MultiCameraSensor::Camera(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);

  if (_index < this->dataPtr->cameras.size())
    return this->dataPtr->cameras[_index];
  else
  {
    gzerr << "camera index out of range. Valid range[0.." <<
        this->dataPtr->cameras.size()-1 << "]\n";
  }
  return rendering::CameraPtr();
}

//////////////////////////////////////////////////
void MultiCameraSensor::SetActive(bool _value)
{
  // If this sensor is reactivated
  if (this->useStrictRate && _value && !this->IsActive())
  {
    // the next rendering time must be reset to ensure it is properly
    // computed by Sensor::NeedsUpdate.
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
  }
  Sensor::SetActive(_value);
}

//////////////////////////////////////////////////
bool MultiCameraSensor::NeedsUpdate()
{
  if (this->useStrictRate)
  {
    double simTime;
    if (this->scene)
      simTime = this->scene->SimTime().Double();
    else
      simTime = this->world->SimTime().Double();

    if (simTime < this->lastMeasurementTime.Double())
    {
      // Rendering sensors also set the lastMeasurementTime variable in Render()
      // and lastUpdateTime in Sensor::Update based on Scene::SimTime() which
      // could be outdated when the world is reset. In this case reset
      // the variables back to 0.
      this->ResetLastUpdateTime();
      return false;
    }

    double dt = this->world->Physics()->GetMaxStepSize();

    // If next rendering time is not set yet
    if (std::isnan(this->dataPtr->nextRenderingTime))
    {
      if (this->updatePeriod == 0
          || (simTime > 0.0 &&
          std::abs(std::fmod(simTime, this->updatePeriod.Double())) < dt))
      {
        this->dataPtr->nextRenderingTime = simTime;
        return true;
      }
      else
      {
        return false;
      }
    }

    if (simTime > this->dataPtr->nextRenderingTime + dt)
      return true;

    // Trigger on the tick the closest from the targeted rendering time
    return (ignition::math::lessOrNearEqual(
          std::abs(simTime - this->dataPtr->nextRenderingTime), dt / 2.0));
  }
  else
  {
    return Sensor::NeedsUpdate();
  }
}

//////////////////////////////////////////////////
void MultiCameraSensor::PrerenderEnded()
{
  if (this->useStrictRate && this->dataPtr->cameras.size() > 0 &&
      this->IsActive() && this->NeedsUpdate())
  {
    // compute next rendering time, take care of the case where period is zero.
    double dt;
    if (this->updatePeriod <= 0.0)
      dt = this->world->Physics()->GetMaxStepSize();
    else
      dt = this->updatePeriod.Double();
    this->dataPtr->nextRenderingTime += dt;

    this->dataPtr->renderNeeded = true;
    this->lastMeasurementTime = this->scene->SimTime();
  }
}

//////////////////////////////////////////////////
void MultiCameraSensor::Render()
{
  if (this->useStrictRate)
  {
    if (!this->dataPtr->renderNeeded)
    {
      return;
    }

    // Update all the cameras
    std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);
    if (this->dataPtr->cameras.empty())
    {
      return;
    }

    for (auto iter = this->dataPtr->cameras.begin();
        iter != this->dataPtr->cameras.end(); ++iter)
    {
      (*iter)->Render();
    }

    this->dataPtr->rendered = true;
    this->dataPtr->renderNeeded = false;
  }
  else
  {
    if (!this->IsActive() || !this->NeedsUpdate())
    {
      return;
    }

    // Update all the cameras
    std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);
    if (this->dataPtr->cameras.empty())
    {
      return;
    }

    for (auto iter = this->dataPtr->cameras.begin();
        iter != this->dataPtr->cameras.end(); ++iter)
    {
      (*iter)->Render();
    }

    this->dataPtr->rendered = true;
    this->lastMeasurementTime = this->scene->SimTime();
  }
}

//////////////////////////////////////////////////
bool MultiCameraSensor::UpdateImpl(const bool /*_force*/)
{
  IGN_PROFILE("MultiCameraSensor::UpdateImpl");
  IGN_PROFILE_BEGIN("Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);

  if (!this->dataPtr->rendered)
    return false;

  bool publish = this->dataPtr->imagePub->HasConnections();

  msgs::Set(this->dataPtr->msg.mutable_time(),
            this->lastMeasurementTime);

  int index = 0;
  for (auto iter = this->dataPtr->cameras.begin();
       iter != this->dataPtr->cameras.end(); ++iter, ++index)
  {
    (*iter)->PostRender();

    if (publish)
    {
      msgs::Image *image = this->dataPtr->msg.mutable_image(index);
      image->set_data((*iter)->ImageData(0),
          image->width() * (*iter)->ImageDepth() * image->height());
    }
  }
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("Publish");
  if (publish)
    this->dataPtr->imagePub->Publish(this->dataPtr->msg);
  IGN_PROFILE_END();

  this->dataPtr->rendered = false;
  return true;
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::CameraCount() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);
  return this->dataPtr->cameras.size();
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::ImageWidth(const unsigned int _index) const
{
  return this->Camera(_index)->ImageWidth();
}

//////////////////////////////////////////////////
unsigned int MultiCameraSensor::ImageHeight(const unsigned int _index) const
{
  return this->Camera(_index)->ImageHeight();
}

//////////////////////////////////////////////////
const unsigned char *MultiCameraSensor::ImageData(const unsigned int _index)
{
  return this->Camera(_index)->ImageData(0);
}

//////////////////////////////////////////////////
bool MultiCameraSensor::SaveFrame(const std::vector<std::string> &_filenames)
{
  this->SetActive(true);

  std::lock_guard<std::mutex> lock(this->dataPtr->cameraMutex);
  if (_filenames.size() != this->dataPtr->cameras.size())
  {
    gzerr << "Filename count[" << _filenames.size() << "] does not match "
          << "camera count[" << this->dataPtr->cameras.size() << "]\n";
    return false;
  }

  bool result = true;

  auto citer = this->dataPtr->cameras.begin();
  for (std::vector<std::string>::const_iterator fiter = _filenames.begin();
       fiter != _filenames.end(); ++fiter, ++citer)
  {
    result &= (*citer)->SaveFrame(*fiter);
  }

  return result;
}

//////////////////////////////////////////////////
bool MultiCameraSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->imagePub && this->dataPtr->imagePub->HasConnections());
}

//////////////////////////////////////////////////
double MultiCameraSensor::NextRequiredTimestamp() const
{
  if (this->useStrictRate)
  {
    if (!ignition::math::equal(this->updatePeriod.Double(), 0.0))
      return this->dataPtr->nextRenderingTime;
    else
      return std::numeric_limits<double>::quiet_NaN();
  }
  else
  {
    return Sensor::NextRequiredTimestamp();
  }
}

//////////////////////////////////////////////////
void MultiCameraSensor::ResetLastUpdateTime()
{
  Sensor::ResetLastUpdateTime();
  if (this->useStrictRate)
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
}
