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
#include <ignition/msgs/Utility.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/CommonIface.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"

#include "gazebo/sensors/CameraSensorPrivate.hh"
#include "gazebo/sensors/CameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("camera", CameraSensor)

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
: Sensor(sensors::IMAGE),
  dataPtr(new CameraSensorPrivate)
{
  this->connections.push_back(
      event::Events::ConnectRender(
        std::bind(&CameraSensor::Render, this)));
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
void CameraSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
  // useStrictRate is set in Sensor::Load()
  if (this->useStrictRate)
  {
    this->connections.push_back(
        event::Events::ConnectPreRenderEnded(
          boost::bind(&CameraSensor::PrerenderEnded, this)));
  }
}

//////////////////////////////////////////////////
std::string CameraSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/image";
  common::replaceAll(topicName, topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
std::string CameraSensor::TopicIgn() const
{
  std::string topicName = this->ScopedName() + "/image";
  common::replaceAll(topicName, topicName, "::", "/");
  common::replaceAll(topicName, topicName, " ", "_");

  return topicName;
}

//////////////////////////////////////////////////
void CameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  // useStrictRate is set in Sensor::Load()
  if (this->useStrictRate)
  {
    this->connections.push_back(
        event::Events::ConnectPreRenderEnded(
          boost::bind(&CameraSensor::PrerenderEnded, this)));
  }

  this->imagePub = this->node->Advertise<msgs::ImageStamped>(this->Topic(), 50);

  ignition::transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(50);
  this->imagePubIgn = this->nodeIgn.Advertise<ignition::msgs::Image>(
      this->TopicIgn(), opts);
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

  std::string worldName = this->world->Name();

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

    std::string scopedName = this->parentName + "::" + this->Name();
    this->camera = this->scene->CreateCamera(scopedName, false);

    if (!this->camera)
    {
      gzerr << "Unable to create camera sensor[mono_camera]\n";
      return;
    }
    this->camera->SetCaptureData(true);

    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    this->camera->Load(cameraSdf);

    // Do some sanity checks
    if (this->camera->ImageWidth() == 0 ||
        this->camera->ImageHeight() == 0)
    {
      gzthrow("image has zero size");
    }

    this->camera->Init();
    this->camera->CreateRenderTexture(scopedName + "_RttTex");
    ignition::math::Pose3d cameraPose = this->pose;
    if (cameraSdf->HasElement("pose"))
      cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

    this->camera->SetWorldPose(cameraPose);
    this->camera->AttachToVisual(this->ParentId(), true, 0, 0);

    if (cameraSdf->HasElement("noise"))
    {
      this->noises[CAMERA_NOISE] =
        NoiseFactory::NewNoiseModel(cameraSdf->GetElement("noise"),
        this->Type());
      this->noises[CAMERA_NOISE]->SetCamera(this->camera);
    }
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

  if (this->camera)
  {
    this->scene->RemoveCamera(this->camera->Name());
  }

  this->camera.reset();

  Sensor::Fini();
}

//////////////////////////////////////////////////
void CameraSensor::SetActive(bool _value)
{
  // If this sensor is reactivated
  if (this->useStrictRate && _value && !this->IsActive())
  {
    // the next rendering time must be reset to ensure it is properly
    // computed by CameraSensor::NeedsUpdate.
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
  }
  Sensor::SetActive(_value);
}

//////////////////////////////////////////////////
bool CameraSensor::NeedsUpdate()
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
void CameraSensor::Update(bool _force)
{
  Sensor::Update(_force);
}

//////////////////////////////////////////////////
void CameraSensor::PrerenderEnded()
{
  if (this->useStrictRate && this->camera && this->IsActive() &&
      this->NeedsUpdate())
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
void CameraSensor::Render()
{
  IGN_PROFILE("sensors::CameraSensor::Render");
  if (this->useStrictRate)
  {
    if (!this->dataPtr->renderNeeded)
      return;

    // Update all the cameras
    this->camera->Render();

    this->dataPtr->rendered = true;
    this->dataPtr->renderNeeded = false;
  }
  else
  {
    if (!this->camera || !this->IsActive() || !this->NeedsUpdate())
      return;

    // Update all the cameras
    this->camera->Render();

    this->dataPtr->rendered = true;
    this->lastMeasurementTime = this->scene->SimTime();
  }
}

//////////////////////////////////////////////////
bool CameraSensor::UpdateImpl(const bool /*_force*/)
{
  IGN_PROFILE("CameraSensor::UpdateImpl");

  if (!this->dataPtr->rendered)
    return false;

  IGN_PROFILE_BEGIN("PostRender");
  this->camera->PostRender();
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("fillarray");

  if ((this->imagePub && this->imagePub->HasConnections()) ||
      this->imagePubIgn.HasConnections())
  {
    auto simTime = this->scene->SimTime();
    if (this->imagePub && this->imagePub->HasConnections())
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), simTime);
      msg.mutable_image()->set_width(this->camera->ImageWidth());
      msg.mutable_image()->set_height(this->camera->ImageHeight());
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
            this->camera->ImageFormat()));

      msg.mutable_image()->set_step(this->camera->ImageWidth() *
          this->camera->ImageDepth());
      msg.mutable_image()->set_data(this->camera->ImageData(),
          msg.image().width() * this->camera->ImageDepth() *
          msg.image().height());

      this->imagePub->Publish(msg);
    }

    if (this->imagePubIgn.HasConnections())
    {
      ignition::msgs::Image msg;
      msg.mutable_header()->mutable_stamp()->set_sec(simTime.sec);
      msg.mutable_header()->mutable_stamp()->set_nsec(simTime.nsec);

      msg.set_width(this->camera->ImageWidth());
      msg.set_height(this->camera->ImageHeight());
      msg.set_pixel_format_type(ignition::msgs::ConvertPixelFormatType(
            this->camera->ImageFormat()));

      msg.set_step(this->camera->ImageWidth() *
          this->camera->ImageDepth());
      msg.set_data(this->camera->ImageData(),
          msg.width() * this->camera->ImageDepth() *
          msg.height());

      this->imagePubIgn.Publish(msg);
    }
  }

  this->dataPtr->rendered = false;
  IGN_PROFILE_END();
  return true;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageWidth() const
{
  if (this->camera)
    return this->camera->ImageWidth();

  if (this->sdf && this->sdf->HasElement("camera"))
  {
    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    sdf::ElementPtr elem = cameraSdf->GetElement("image");
    return elem->Get<unsigned int>("width");
  }

  gzwarn << "Can't get image width." << std::endl;
  return 0;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageHeight() const
{
  if (this->camera)
    return this->camera->ImageHeight();

  if (this->sdf && this->sdf->HasElement("camera"))
  {
    sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
    sdf::ElementPtr elem = cameraSdf->GetElement("image");
    return elem->Get<unsigned int>("height");
  }

  gzwarn << "Can't get image height." << std::endl;
  return 0;
}

//////////////////////////////////////////////////
const unsigned char *CameraSensor::ImageData() const
{
  if (this->camera)
    return this->camera->ImageData(0);
  else
    return nullptr;
}

//////////////////////////////////////////////////
bool CameraSensor::SaveFrame(const std::string &_filename)
{
  this->SetActive(true);

  if (this->camera)
    return this->camera->SaveFrame(_filename);
  else
    return false;
}

//////////////////////////////////////////////////
bool CameraSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->imagePub && this->imagePub->HasConnections()) ||
    this->imagePubIgn.HasConnections();
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::Camera() const
{
  return this->camera;
}

//////////////////////////////////////////////////
bool CameraSensor::Rendered() const
{
  return this->dataPtr->rendered;
}

//////////////////////////////////////////////////
void CameraSensor::SetRendered(const bool _value)
{
  this->dataPtr->rendered = _value;
}

//////////////////////////////////////////////////
double CameraSensor::NextRequiredTimestamp() const
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
void CameraSensor::ResetLastUpdateTime()
{
  Sensor::ResetLastUpdateTime();
  if (this->useStrictRate)
    this->dataPtr->nextRenderingTime = std::numeric_limits<double>::quiet_NaN();
}
