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
#include "gazebo/common/Profiler.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/common/Timer.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Plugin.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Distortion.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/sensors/LogicalCameraSensor.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorPrivate.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

sdf::ElementPtr SensorPrivate::sdfSensor;

bool Sensor::useStrictRate = false;

//////////////////////////////////////////////////
Sensor::Sensor(SensorCategory _cat)
: dataPtr(new SensorPrivate)
{
  this->dataPtr->extension = std::make_shared<SensorExt>(this);
  this->SetExtension(this->dataPtr->extension);

  if (!this->dataPtr->sdfSensor)
  {
    this->dataPtr->sdfSensor.reset(new sdf::Element);
    sdf::initFile("sensor.sdf", this->dataPtr->sdfSensor);
  }

  this->dataPtr->category = _cat;

  this->sdf = this->dataPtr->sdfSensor->Clone();

  this->active = false;

  this->node = transport::NodePtr(new transport::Node());

  this->dataPtr->updateDelay = common::Time(0.0);
  this->updatePeriod = common::Time(0.0);

  this->dataPtr->id = physics::getUniqueId();
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Sensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
  this->Load(_worldName);
}

//////////////////////////////////////////////////
void Sensor::Load(const std::string &_worldName)
{
  if (this->sdf->HasElement("pose"))
  {
    this->pose =
      this->sdf->Get<ignition::math::Pose3d>("pose");
  }

  if (this->sdf->Get<bool>("always_on"))
    this->SetActive(true);

  this->useStrictRate = rendering::lockstep_enabled();

  this->world = physics::get_world(_worldName);

  if (this->dataPtr->category == IMAGE)
  {
    this->scene = rendering::get_scene(_worldName);
  }

  // loaded, but not updated
  this->lastUpdateTime = common::Time(0.0);

  this->node->Init(this->world->Name());
  this->dataPtr->sensorPub =
    this->node->Advertise<msgs::Sensor>("~/sensor");
}

//////////////////////////////////////////////////
void Sensor::Init()
{
  this->SetUpdateRate(this->sdf->Get<double>("update_rate"));

  // Load the plugins
  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  msgs::Sensor msg;
  this->FillMsg(msg);
  this->dataPtr->sensorPub->Publish(msg);
}

//////////////////////////////////////////////////
void Sensor::SetParent(const std::string &_name, const uint32_t _id)
{
  this->parentName = _name;
  this->parentId = _id;
}

//////////////////////////////////////////////////
std::string Sensor::ParentName() const
{
  return this->parentName;
}

//////////////////////////////////////////////////
uint32_t Sensor::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
uint32_t Sensor::ParentId() const
{
  return this->parentId;
}

//////////////////////////////////////////////////
bool Sensor::NeedsUpdate()
{
  return this->dataPtr->extension->NeedsUpdate();
}

//////////////////////////////////////////////////
void Sensor::Update(const bool _force)
{
  if (this->IsActive() || _force)
  {
    if (this->useStrictRate)
    {
      // rendering sensors (IMAGE category) has it's own mechanism
      // for throttling and lockstepping with physics. So throttle just
      // physics sensors
      if (this->dataPtr->category != IMAGE && !this->NeedsUpdate() && !_force)
        return;

      if (this->UpdateImpl(_force))
      {
        common::Time simTime = this->world->SimTime();
        this->lastUpdateTime = simTime;
        this->dataPtr->updated();
      }
    }
    else
    {
      common::Time simTime;
      if (this->dataPtr->category == IMAGE && this->scene)
        simTime = this->scene->SimTime();
      else
        simTime = this->world->SimTime();

      {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutexLastUpdateTime);

        if (simTime <= this->lastUpdateTime && !_force)
          return;

        // Adjust time-to-update period to compensate for delays caused by
        // another sensor's update in the same thread.
        // NOTE: If you change this equation, also change the matching equation
        // in Sensor::NeedsUpdate
        common::Time adjustedElapsed = simTime -
          this->lastUpdateTime + this->dataPtr->updateDelay;

        if (adjustedElapsed < this->updatePeriod && !_force)
          return;

        this->dataPtr->updateDelay = std::max(common::Time::Zero,
            adjustedElapsed - this->updatePeriod);

        // if delay is more than a full update period, then give up trying
        // to catch up. This happens normally when the sensor just changed from
        // an inactive to an active state, or the sensor just cannot hit its
        // target update rate (worst case).
        if (this->dataPtr->updateDelay >= this->updatePeriod)
          this->dataPtr->updateDelay = common::Time::Zero;
      }

      if (this->UpdateImpl(_force))
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutexLastUpdateTime);
        this->lastUpdateTime = simTime;
        this->dataPtr->updated();
      }
    }
  }
}

//////////////////////////////////////////////////
void Sensor::Fini()
{
  if (this->node)
    this->node->Fini();
  this->node.reset();

  this->connections.clear();

  for (auto &it : this->noises)
    it.second->Fini();
  this->noises.clear();

  this->active = false;
  this->plugins.clear();

  if (this->sdf)
    this->sdf->Reset();
  this->sdf.reset();

  this->scene.reset();
  this->world.reset();
}

//////////////////////////////////////////////////
std::string Sensor::Name() const
{
  if (this->sdf)
    return this->sdf->Get<std::string>("name");

  gzwarn << "Missing sensor SDF." << std::endl;
  return "";
}

//////////////////////////////////////////////////
std::string Sensor::ScopedName() const
{
  return this->world->Name() + "::" + this->parentName + "::" + this->Name();
}

//////////////////////////////////////////////////
void Sensor::LoadPlugin(sdf::ElementPtr _sdf)
{
  std::string name = _sdf->Get<std::string>("name");
  std::string filename = _sdf->Get<std::string>("filename");
  gazebo::SensorPluginPtr plugin = gazebo::SensorPlugin::Create(filename, name);

  if (plugin)
  {
    if (plugin->GetType() != SENSOR_PLUGIN)
    {
      gzerr << "Sensor[" << this->Name() << "] is attempting to load "
            << "a plugin, but detected an incorrect plugin type. "
            << "Plugin filename[" << filename << "] name[" << name << "]\n";
      return;
    }

    SensorPtr myself = shared_from_this();
    plugin->Load(myself, _sdf);
    plugin->Init();
    this->plugins.push_back(plugin);
  }
}

//////////////////////////////////////////////////
void Sensor::SetActive(const bool _value)
{
  this->dataPtr->extension->SetActive(_value);
}

//////////////////////////////////////////////////
bool Sensor::IsActive() const
{
  return this->active;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Sensor::Pose() const
{
  return this->pose;
}

//////////////////////////////////////////////////
void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->pose = _pose;

  // Update the visualization with the pose information.
  if (this->dataPtr->sensorPub && this->Visualize())
  {
    msgs::Sensor msg;
    msg.set_name(this->Name());
    msg.set_id(this->Id());
    msg.set_parent(this->ParentName());
    msg.set_parent_id(this->ParentId());
    msg.set_type(this->Type());
    msg.set_visualize(true);
    msgs::Set(msg.mutable_pose(), this->pose);
    this->dataPtr->sensorPub->Publish(msg);
  }
}

//////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  if (this->updatePeriod.Double() > 0.0)
    return 1.0/this->updatePeriod.Double();
  else
    return 0.0;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(const double _hz)
{
  if (_hz > 0.0)
    this->updatePeriod = 1.0/_hz;
  else
    this->updatePeriod = 0.0;
}

//////////////////////////////////////////////////
common::Time Sensor::LastUpdateTime() const
{
  return this->lastUpdateTime;
}

//////////////////////////////////////////////////
common::Time Sensor::LastMeasurementTime() const
{
  return this->lastMeasurementTime;
}

//////////////////////////////////////////////////
std::string Sensor::Type() const
{
  return this->sdf->Get<std::string>("type");
}

//////////////////////////////////////////////////
bool Sensor::Visualize() const
{
  return this->sdf->Get<bool>("visualize");
}

//////////////////////////////////////////////////
std::string Sensor::Topic() const
{
  std::string result;
  if (this->sdf->HasElement("topic") &&
      this->sdf->Get<std::string>("topic") != "__default__")
    result = this->sdf->Get<std::string>("topic");
  return result;
}

//////////////////////////////////////////////////
void Sensor::FillMsg(msgs::Sensor &_msg)
{
  _msg.set_name(this->Name());
  _msg.set_id(this->Id());
  _msg.set_type(this->Type());
  _msg.set_parent(this->ParentName());
  _msg.set_parent_id(this->ParentId());
  msgs::Set(_msg.mutable_pose(), this->Pose());

  _msg.set_always_on(this->IsActive());
  _msg.set_topic(this->Topic());
  _msg.set_update_rate(this->UpdateRate());
  _msg.set_visualize(this->Visualize());

  if (this->Type() == "logical_camera")
  {
    LogicalCameraSensor *camSensor = static_cast<LogicalCameraSensor*>(this);
    msgs::LogicalCameraSensor *camMsg = _msg.mutable_logical_camera();
    camMsg->set_near_clip(camSensor->Near());
    camMsg->set_far_clip(camSensor->Far());
    camMsg->set_horizontal_fov(camSensor->HorizontalFOV().Radian());
    camMsg->set_aspect_ratio(camSensor->AspectRatio());
  }
  else if (this->Type() == "camera" || this->Type() == "wideanglecamera")
  {
    CameraSensor *camSensor = static_cast<CameraSensor*>(this);
    msgs::CameraSensor *camMsg = _msg.mutable_camera();
    auto cam = camSensor->Camera();
    camMsg->set_horizontal_fov(cam->HFOV().Radian());
    camMsg->mutable_image_size()->set_x(camSensor->ImageWidth());
    camMsg->mutable_image_size()->set_y(camSensor->ImageHeight());
    camMsg->set_image_format(cam->ImageFormat());
    camMsg->set_near_clip(cam->NearClip());
    camMsg->set_far_clip(cam->FarClip());
    auto distortion = cam->LensDistortion();
    if (distortion)
    {
      msgs::Distortion *distortionMsg = camMsg->mutable_distortion();
      distortionMsg->set_k1(distortion->K1());
      distortionMsg->set_k2(distortion->K2());
      distortionMsg->set_k3(distortion->K3());
      distortionMsg->set_p1(distortion->P1());
      distortionMsg->set_p2(distortion->P2());
      distortionMsg->mutable_center()->set_x(distortion->Center().X());
      distortionMsg->mutable_center()->set_y(distortion->Center().Y());
    }
  }
}

//////////////////////////////////////////////////
std::string Sensor::WorldName() const
{
  return this->world->Name();
}

//////////////////////////////////////////////////
SensorCategory Sensor::Category() const
{
  return this->dataPtr->category;
}

//////////////////////////////////////////////////
NoisePtr Sensor::Noise(const SensorNoiseType _type) const
{
  if (this->noises.find(_type) == this->noises.end())
  {
    gzerr << "Get noise index not valid" << std::endl;
    return NoisePtr();
  }
  return this->noises.at(_type);
}

//////////////////////////////////////////////////
void Sensor::ResetLastUpdateTime()
{
  this->dataPtr->extension->ResetLastUpdateTime();
}

//////////////////////////////////////////////////
event::ConnectionPtr Sensor::ConnectUpdated(std::function<void()> _subscriber)
{
  return this->dataPtr->updated.Connect(_subscriber);
}

//////////////////////////////////////////////////
double Sensor::NextRequiredTimestamp() const
{
  return this->dataPtr->extension->NextRequiredTimestamp();
}

//////////////////////////////////////////////////
bool Sensor::StrictRate() const
{
  return this->useStrictRate;
}

//////////////////////////////////////////////////
void Sensor::SetExtension(std::shared_ptr<SensorExt> _ext)
{
  this->dataPtr->extension = _ext;
}

//////////////////////////////////////////////////
SensorExt::SensorExt(Sensor *_sensor)
{
  this->sensor = _sensor;
}

//////////////////////////////////////////////////
void SensorExt::SetActive(bool _value)
{
  this->sensor->active = _value;
}

//////////////////////////////////////////////////
void SensorExt::ResetLastUpdateTime()
{
  std::lock_guard<std::mutex> lock(sensor->dataPtr->mutexLastUpdateTime);
  this->sensor->lastUpdateTime = 0.0;
  this->sensor->lastMeasurementTime = 0.0;
  this->sensor->dataPtr->updateDelay = 0.0;
}

//////////////////////////////////////////////////
double SensorExt::NextRequiredTimestamp() const
{
  // implementation by default: next required timestamp is ignored
  return std::numeric_limits<double>::quiet_NaN();
}

//////////////////////////////////////////////////
bool SensorExt::NeedsUpdate()
{
  // Adjust time-to-update period to compensate for delays caused by another
  // sensor's update in the same thread.

  common::Time simTime;
  if (this->sensor->dataPtr->category == IMAGE && this->sensor->scene)
    simTime = sensor->scene->SimTime();
  else
    simTime = this->sensor->world->SimTime();

  // case when last update occurred in the future probably due to
  // world reset
  if (simTime <= this->sensor->lastMeasurementTime)
  {
    // Rendering sensors also set the lastMeasurementTime variable in Render()
    // and lastUpdateTime in Sensor::Update based on Scene::SimTime() which
    // could be outdated when the world is reset. In this case reset
    // the variables back to 0.
    this->sensor->ResetLastUpdateTime();
    return false;
  }

  return (simTime - this->sensor->lastMeasurementTime +
      this->sensor->dataPtr->updateDelay) >= this->sensor->updatePeriod;
}

//////////////////////////////////////////////////
RenderingSensorExt::RenderingSensorExt(Sensor *_sensor)
  : SensorExt(_sensor), dataPtr(new RenderingSensorExtPrivate)
{
}

//////////////////////////////////////////////////
void RenderingSensorExt::SetActive(bool _value)
{
  // If this sensor is reactivated
  if (this->sensor->useStrictRate && _value && !this->sensor->IsActive())
  {
    // the next rendering time must be reset to ensure it is properly
    // computed by CameraSensor::NeedsUpdate.
    this->dataPtr->nextRenderingTime =
        std::numeric_limits<double>::quiet_NaN();
  }
  SensorExt::SetActive(_value);
}

//////////////////////////////////////////////////
void RenderingSensorExt::ResetLastUpdateTime()
{
  SensorExt::ResetLastUpdateTime();
  if (this->sensor->useStrictRate)
  {
    this->dataPtr->nextRenderingTime =
      std::numeric_limits<double>::quiet_NaN();
  }
}

//////////////////////////////////////////////////
double RenderingSensorExt::NextRequiredTimestamp() const
{
  if (this->sensor->useStrictRate)
  {
    if (!ignition::math::equal(this->sensor->updatePeriod.Double(), 0.0))
      return this->dataPtr->nextRenderingTime;
    else
      return std::numeric_limits<double>::quiet_NaN();
  }
  else
  {
    return SensorExt::NextRequiredTimestamp();
  }
}

//////////////////////////////////////////////////
bool RenderingSensorExt::NeedsUpdate()
{
  if (this->sensor->useStrictRate)
  {
    double simTime;
    if (this->sensor->scene)
      simTime = this->sensor->scene->SimTime().Double();
    else
      simTime = this->sensor->world->SimTime().Double();

    if (simTime < this->sensor->lastMeasurementTime.Double())
    {
      // Rendering sensors also set the lastMeasurementTime variable in Render()
      // and lastUpdateTime in Sensor::Update based on Scene::SimTime() which
      // could be outdated when the world is reset. In this case reset
      // the variables back to 0.
      this->ResetLastUpdateTime();
      return false;
    }

    double dt = this->sensor->world->Physics()->GetMaxStepSize();

    // If next rendering time is not set yet
    if (std::isnan(this->dataPtr->nextRenderingTime))
    {
      if (this->sensor->updatePeriod == 0
          || (simTime > 0.0 &&
          std::abs(std::fmod(simTime, this->sensor->updatePeriod.Double()))
          < dt))
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
    return SensorExt::NeedsUpdate();
  }
}

//////////////////////////////////////////////////
void RenderingSensorExt::SetNextRenderingTime(double _t)
{
  this->dataPtr->nextRenderingTime = _t;
}

//////////////////////////////////////////////////
double RenderingSensorExt::NextRenderingTime() const
{
  return this->dataPtr->nextRenderingTime;
}
