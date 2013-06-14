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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 */

#include "sdf/sdf.hh"
#include "transport/transport.hh"

#include "physics/Physics.hh"
#include "physics/World.hh"

#include "common/Timer.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Plugin.hh"

#include "sensors/CameraSensor.hh"

#include "sensors/Sensor.hh"
#include "sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
Sensor::Sensor(SensorCategory _cat)
{
  this->category = _cat;

  this->sdf.reset(new sdf::Element);
  sdf::initFile("sensor.sdf", this->sdf);

  this->active = false;

  this->node = transport::NodePtr(new transport::Node());

  this->updateDelay = common::Time(0.0);
  this->updatePeriod = common::Time(0.0);
}

//////////////////////////////////////////////////
Sensor::~Sensor()
{
  if (this->node)
    this->node->Fini();
  this->node.reset();

  if (this->sdf)
    this->sdf->Reset();
  this->sdf.reset();
  this->connections.clear();
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
    this->pose = this->sdf->GetValuePose("pose");
  }

  if (this->sdf->GetValueBool("always_on"))
    this->SetActive(true);

  this->world = physics::get_world(_worldName);

  // loaded, but not updated
  this->lastUpdateTime = common::Time(0.0);

  this->node->Init(this->world->GetName());
  this->sensorPub = this->node->Advertise<msgs::Sensor>("~/sensor");
}

//////////////////////////////////////////////////
void Sensor::Init()
{
  this->SetUpdateRate(this->sdf->GetValueDouble("update_rate"));

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
  this->sensorPub->Publish(msg);
}

//////////////////////////////////////////////////
void Sensor::SetParent(const std::string &_name)
{
  this->parentName = _name;
}

//////////////////////////////////////////////////
std::string Sensor::GetParentName() const
{
  return this->parentName;
}

//////////////////////////////////////////////////
void Sensor::Update(bool _force)
{
  if (this->IsActive() || _force)
  {
    // Adjust time-to-update period to compensate for delays caused by another
    // sensor's update in the same thread.
    common::Time adjustedElapsed = this->world->GetSimTime() -
        this->lastUpdateTime + this->updateDelay;

    if (adjustedElapsed >= this->updatePeriod || _force)
    {
      this->updateDelay = std::max(common::Time::Zero,
          adjustedElapsed - this->updatePeriod);

      // if delay is more than a full update period, then give up trying
      // to catch up. This happens normally when the sensor just changed from
      // an inactive to an active state, or the sensor just cannot hit its
      // target update rate (worst case).
      if (this->updateDelay >= this->updatePeriod)
        this->updateDelay = common::Time::Zero;

      this->lastUpdateTime = this->world->GetSimTime();
      this->UpdateImpl(_force);
      this->updated();
    }
  }
}

//////////////////////////////////////////////////
void Sensor::Fini()
{
  this->active = false;
  this->plugins.clear();
}

//////////////////////////////////////////////////
std::string Sensor::GetName() const
{
  return this->sdf->GetValueString("name");
}

//////////////////////////////////////////////////
std::string Sensor::GetScopedName() const
{
  return this->world->GetName() + "::" + this->parentName + "::" +
         this->GetName();
}

//////////////////////////////////////////////////
void Sensor::LoadPlugin(sdf::ElementPtr _sdf)
{
  std::string name = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  gazebo::SensorPluginPtr plugin = gazebo::SensorPlugin::Create(filename, name);

  if (plugin)
  {
    if (plugin->GetType() != SENSOR_PLUGIN)
    {
      gzerr << "Sensor[" << this->GetName() << "] is attempting to load "
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
void Sensor::SetActive(bool _value)
{
  this->active = _value;
}

//////////////////////////////////////////////////
bool Sensor::IsActive()
{
  return this->active;
}

//////////////////////////////////////////////////
math::Pose Sensor::GetPose() const
{
  return this->pose;
}

//////////////////////////////////////////////////
double Sensor::GetUpdateRate()
{
  if (this->updatePeriod.Double() > 0.0)
    return 1.0/this->updatePeriod.Double();
  else
    return 0.0;
}

//////////////////////////////////////////////////
void Sensor::SetUpdateRate(double _hz)
{
  if (_hz > 0.0)
    this->updatePeriod = 1.0/_hz;
  else
    this->updatePeriod = 0.0;
}

//////////////////////////////////////////////////
common::Time Sensor::GetLastUpdateTime()
{
  return this->lastUpdateTime;
}

//////////////////////////////////////////////////
common::Time Sensor::GetLastMeasurementTime()
{
  return this->lastMeasurementTime;
}

//////////////////////////////////////////////////
std::string Sensor::GetType() const
{
  return this->sdf->GetValueString("type");
}

//////////////////////////////////////////////////
bool Sensor::GetVisualize() const
{
  return this->sdf->GetValueBool("visualize");
}

//////////////////////////////////////////////////
std::string Sensor::GetTopic() const
{
  std::string result;
  if (this->sdf->HasElement("topic") &&
      this->sdf->GetValueString("topic") != "__default__")
    result = this->sdf->GetValueString("topic");
  return result;
}

//////////////////////////////////////////////////
void Sensor::FillMsg(msgs::Sensor &_msg)
{
  _msg.set_name(this->GetName());
  _msg.set_type(this->GetType());
  _msg.set_parent(this->GetParentName());
  msgs::Set(_msg.mutable_pose(), this->GetPose());

  _msg.set_visualize(this->GetVisualize());
  _msg.set_topic(this->GetTopic());

  if (this->GetType() == "camera")
  {
    CameraSensor *camSensor = static_cast<CameraSensor*>(this);
    msgs::CameraSensor *camMsg = _msg.mutable_camera();
    camMsg->mutable_image_size()->set_x(camSensor->GetImageWidth());
    camMsg->mutable_image_size()->set_y(camSensor->GetImageHeight());
  }
}

//////////////////////////////////////////////////
std::string Sensor::GetWorldName() const
{
  return this->world->GetName();
}

//////////////////////////////////////////////////
SensorCategory Sensor::GetCategory() const
{
  return this->category;
}

//////////////////////////////////////////////////
void Sensor::ResetLastUpdateTime()
{
  this->lastUpdateTime = 0.0;
}
