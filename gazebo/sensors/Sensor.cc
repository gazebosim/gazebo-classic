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

#include <sdf/sdf.hh>

#include "gazebo/transport/transport.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"

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

//////////////////////////////////////////////////
Sensor::Sensor(SensorCategory _cat)
: dPtr(new SensorProtected), pdPtr(new SensorPrivate)
{
  this->ConstructorHelper(_cat);
}

//////////////////////////////////////////////////
Sensor::Sensor(SensorProtected &_dataPtr, SensorCategory _cat)
: dPtr(&_dataPtr), pdPtr(new SensorPrivate)
{
  this->ConstructorHelper(_cat);
}

//////////////////////////////////////////////////
void Sensor::ConstructorHelper(SensorCategory _cat)
{
  if (!this->pdPtr->sdfSensor)
  {
    this->pdPtr->sdfSensor.reset(new sdf::Element);
    sdf::initFile("sensor.sdf", this->pdPtr->sdfSensor);
  }

  this->pdPtr->category = _cat;

  this->dPtr->sdf = this->pdPtr->sdfSensor->Clone();

  this->dPtr->active = false;

  this->dPtr->node = transport::NodePtr(new transport::Node());

  this->pdPtr->updateDelay = common::Time(0.0);
  this->dPtr->updatePeriod = common::Time(0.0);

  this->pdPtr->id = physics::getUniqueId();
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
  if (this->dPtr->node)
    this->dPtr->node->Fini();
  this->dPtr->node.reset();

  if (this->dPtr->sdf)
    this->dPtr->sdf->Reset();
  this->dPtr->sdf.reset();
  this->dPtr->connections.clear();
  this->dPtr->noises.clear();
}

//////////////////////////////////////////////////
void Sensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  this->dPtr->sdf->Copy(_sdf);
  this->Load(_worldName);
}

//////////////////////////////////////////////////
void Sensor::Load(const std::string &_worldName)
{
  if (this->dPtr->sdf->HasElement("pose"))
  {
    this->dPtr->pose = this->dPtr->sdf->Get<ignition::math::Pose3d>("pose");
  }

  if (this->dPtr->sdf->Get<bool>("always_on"))
    this->SetActive(true);

  this->dPtr->world = physics::get_world(_worldName);

  if (this->pdPtr->category == IMAGE)
    this->dPtr->scene = rendering::get_scene(_worldName);

  // loaded, but not updated
  this->dPtr->lastUpdateTime = common::Time(0.0);

  this->dPtr->node->Init(this->dPtr->world->GetName());
  this->pdPtr->sensorPub =
    this->dPtr->node->Advertise<msgs::Sensor>("~/sensor");
}

//////////////////////////////////////////////////
void Sensor::Init()
{
  this->SetUpdateRate(this->dPtr->sdf->Get<double>("update_rate"));

  // Load the plugins
  if (this->dPtr->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->dPtr->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  msgs::Sensor msg;
  this->FillMsg(msg);
  this->pdPtr->sensorPub->Publish(msg);
}

//////////////////////////////////////////////////
void Sensor::SetParent(const std::string &_name, const uint32_t _id)
{
  this->dPtr->parentName = _name;
  this->dPtr->parentId = _id;
}

//////////////////////////////////////////////////
std::string Sensor::GetParentName() const
{
  return this->ParentName();
}

//////////////////////////////////////////////////
std::string Sensor::ParentName() const
{
  return this->dPtr->parentName;
}

//////////////////////////////////////////////////
uint32_t Sensor::GetId() const
{
  return this->Id();
}

//////////////////////////////////////////////////
uint32_t Sensor::Id() const
{
  return this->pdPtr->id;
}

//////////////////////////////////////////////////
uint32_t Sensor::GetParentId() const
{
  return this->ParentId();
}

//////////////////////////////////////////////////
uint32_t Sensor::ParentId() const
{
  return this->dPtr->parentId;
}

//////////////////////////////////////////////////
bool Sensor::NeedsUpdate()
{
  // Adjust time-to-update period to compensate for delays caused by another
  // sensor's update in the same thread.

  common::Time simTime;
  if (this->pdPtr->category == IMAGE && this->dPtr->scene)
    simTime = this->dPtr->scene->GetSimTime();
  else
    simTime = this->dPtr->world->GetSimTime();

  if (simTime <= this->dPtr->lastMeasurementTime)
    return false;

  return (simTime - this->dPtr->lastMeasurementTime +
      this->pdPtr->updateDelay) >= this->dPtr->updatePeriod;
}

//////////////////////////////////////////////////
void Sensor::Update(const bool _force)
{
  if (this->IsActive() || _force)
  {
    common::Time simTime;
    if (this->pdPtr->category == IMAGE && this->dPtr->scene)
      simTime = this->dPtr->scene->GetSimTime();
    else
      simTime = this->dPtr->world->GetSimTime();

    {
      std::lock_guard<std::mutex> lock(this->pdPtr->mutexLastUpdateTime);

      if (simTime <= this->dPtr->lastUpdateTime && !_force)
        return;

      // Adjust time-to-update period to compensate for delays caused by another
      // sensor's update in the same thread.
      // NOTE: If you change this equation, also change the matching equation in
      // Sensor::NeedsUpdate
      common::Time adjustedElapsed = simTime - this->dPtr->lastUpdateTime +
        this->pdPtr->updateDelay;

      if (adjustedElapsed < this->dPtr->updatePeriod && !_force)
        return;

      this->pdPtr->updateDelay = std::max(common::Time::Zero,
          adjustedElapsed - this->dPtr->updatePeriod);

      // if delay is more than a full update period, then give up trying
      // to catch up. This happens normally when the sensor just changed from
      // an inactive to an active state, or the sensor just cannot hit its
      // target update rate (worst case).
      if (this->pdPtr->updateDelay >= this->dPtr->updatePeriod)
        this->pdPtr->updateDelay = common::Time::Zero;
    }

    if (this->UpdateImpl(_force))
    {
      std::lock_guard<std::mutex> lock(this->pdPtr->mutexLastUpdateTime);
      this->dPtr->lastUpdateTime = simTime;
      this->pdPtr->updated();
    }
  }
}

//////////////////////////////////////////////////
void Sensor::Fini()
{
  for (auto &it : this->dPtr->noises)
    it.second->Fini();

  this->dPtr->active = false;
  this->dPtr->plugins.clear();
}

//////////////////////////////////////////////////
std::string Sensor::GetName() const
{
  return this->Name();
}

//////////////////////////////////////////////////
std::string Sensor::Name() const
{
  return this->dPtr->sdf->Get<std::string>("name");
}

//////////////////////////////////////////////////
std::string Sensor::GetScopedName() const
{
  return this->ScopedName();
}

//////////////////////////////////////////////////
std::string Sensor::ScopedName() const
{
  return this->dPtr->world->GetName() + "::" + this->dPtr->parentName + "::" +
    this->Name();
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
    this->dPtr->plugins.push_back(plugin);
  }
}

//////////////////////////////////////////////////
void Sensor::SetActive(const bool _value)
{
  this->dPtr->active = _value;
}

//////////////////////////////////////////////////
bool Sensor::IsActive() const
{
  return this->dPtr->active;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Sensor::Pose() const
{
  return this->dPtr->pose;
}

//////////////////////////////////////////////////
void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dPtr->pose = _pose;

  // Update the visualization with the pose information.
  if (this->pdPtr->sensorPub && this->Visualize())
  {
    msgs::Sensor msg;
    msg.set_name(this->Name());
    msg.set_id(this->Id());
    msg.set_parent(this->ParentName());
    msg.set_parent_id(this->ParentId());
    msg.set_type(this->Type());
    msg.set_visualize(true);
    msgs::Set(msg.mutable_pose(), this->dPtr->pose);
    this->pdPtr->sensorPub->Publish(msg);
  }
}

//////////////////////////////////////////////////
double Sensor::GetUpdateRate()
{
  return this->UpdateRate();
}

//////////////////////////////////////////////////
double Sensor::UpdateRate() const
{
  if (this->dPtr->updatePeriod.Double() > 0.0)
    return 1.0/this->dPtr->updatePeriod.Double();
  else
    return 0.0;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(const double _hz)
{
  if (_hz > 0.0)
    this->dPtr->updatePeriod = 1.0/_hz;
  else
    this->dPtr->updatePeriod = 0.0;
}

//////////////////////////////////////////////////
common::Time Sensor::GetLastUpdateTime()
{
  return this->LastUpdateTime();
}

//////////////////////////////////////////////////
common::Time Sensor::LastUpdateTime() const
{
  return this->dPtr->lastUpdateTime;
}

//////////////////////////////////////////////////
common::Time Sensor::GetLastMeasurementTime()
{
  return this->LastMeasurementTime();
}

//////////////////////////////////////////////////
common::Time Sensor::LastMeasurementTime() const
{
  return this->dPtr->lastMeasurementTime;
}

//////////////////////////////////////////////////
std::string Sensor::GetType() const
{
  return this->Type();
}

//////////////////////////////////////////////////
std::string Sensor::Type() const
{
  return this->dPtr->sdf->Get<std::string>("type");
}

//////////////////////////////////////////////////
bool Sensor::GetVisualize() const
{
  return this->Visualize();
}

//////////////////////////////////////////////////
bool Sensor::Visualize() const
{
  return this->dPtr->sdf->Get<bool>("visualize");
}

//////////////////////////////////////////////////
std::string Sensor::GetTopic() const
{
  return this->Topic();
}

//////////////////////////////////////////////////
std::string Sensor::Topic() const
{
  std::string result;
  if (this->dPtr->sdf->HasElement("topic") &&
      this->dPtr->sdf->Get<std::string>("topic") != "__default__")
    result = this->dPtr->sdf->Get<std::string>("topic");
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
    camMsg->set_horizontal_fov(cam->GetHFOV().Radian());
    camMsg->mutable_image_size()->set_x(camSensor->ImageWidth());
    camMsg->mutable_image_size()->set_y(camSensor->ImageHeight());
    camMsg->set_image_format(cam->GetImageFormat());
    camMsg->set_near_clip(cam->GetNearClip());
    camMsg->set_far_clip(cam->GetFarClip());
    auto distortion = cam->GetDistortion();
    if (distortion)
    {
      msgs::Distortion *distortionMsg = camMsg->mutable_distortion();
      distortionMsg->set_k1(distortion->GetK1());
      distortionMsg->set_k2(distortion->GetK2());
      distortionMsg->set_k3(distortion->GetK3());
      distortionMsg->set_p1(distortion->GetP1());
      distortionMsg->set_p2(distortion->GetP2());
      distortionMsg->mutable_center()->set_x(distortion->GetCenter().x);
      distortionMsg->mutable_center()->set_y(distortion->GetCenter().y);
    }
  }
}

//////////////////////////////////////////////////
std::string Sensor::GetWorldName() const
{
  return this->WorldName();
}

//////////////////////////////////////////////////
std::string Sensor::WorldName() const
{
  return this->dPtr->world->GetName();
}

//////////////////////////////////////////////////
SensorCategory Sensor::GetCategory() const
{
  return this->Category();
}

//////////////////////////////////////////////////
SensorCategory Sensor::Category() const
{
  return this->pdPtr->category;
}

//////////////////////////////////////////////////
NoisePtr Sensor::GetNoise(const SensorNoiseType _type) const
{
  return this->Noise(_type);
}

//////////////////////////////////////////////////
NoisePtr Sensor::Noise(const SensorNoiseType _type) const
{
  if (this->dPtr->noises.find(_type) == this->dPtr->noises.end())
  {
    gzerr << "Get noise index not valid" << std::endl;
    return NoisePtr();
  }
  return this->dPtr->noises.at(_type);
}

//////////////////////////////////////////////////
void Sensor::ResetLastUpdateTime()
{
  std::lock_guard<std::mutex> lock(this->pdPtr->mutexLastUpdateTime);
  this->dPtr->lastUpdateTime = 0.0;
}

//////////////////////////////////////////////////
event::ConnectionPtr Sensor::ConnectUpdated(std::function<void()> _subscriber)
{
  return this->pdPtr->updated.Connect(_subscriber);
}

//////////////////////////////////////////////////
void Sensor::DisconnectUpdated(event::ConnectionPtr &_c)
{
  this->pdPtr->updated.Disconnect(_c);
}

