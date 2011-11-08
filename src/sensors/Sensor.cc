/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "sdf/sdf_parser.h"
#include "transport/transport.h"

#include "physics/Physics.hh"
#include "physics/World.hh"

#include "common/Timer.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Plugin.hh"

#include "sensors/Sensor.hh"
#include "sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor()
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile( "/sdf/sensor.sdf", this->sdf );

  this->active = false;

  this->node = transport::NodePtr(new transport::Node());

  this->updatePeriod = common::Time(0.0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
  this->sdf->Reset();
  this->sdf.reset();
  this->node.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load( sdf::ElementPtr &_sdf )
{
  this->sdf = _sdf;
  this->Load();
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load()
{
  if (this->sdf->HasElement("origin"))
  {
    this->pose =  this->sdf->GetElement("origin")->GetValuePose("pose");
  }

  this->node->Init(this->sdf->GetWorldName());

  // get world
  std::string worldName = this->sdf->GetWorldName();
  this->world = physics::get_world(worldName);
  this->lastUpdateTime = this->world->GetSimTime(); //TODO: hmm, special case for start.
}
 
////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
  //gzerr << "Sensor::Init()\n";

  this->SetUpdateRate( this->sdf->GetValueDouble("update_rate")  );

  // Load the plugins
  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = this->sdf->GetNextElement("plugin", pluginElem);
    }
  }
  //SensorManager::Instance()->AddSensor(shared_from_this());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the parent of the sensor
void Sensor::SetParent( const std::string &_name )
{
  this->parentName = _name;
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update(bool _force)
{
  if (this->IsActive())
  {
    if (this->world->GetSimTime() - this->lastUpdateTime >= this->updatePeriod)
    {
      this->UpdateImpl(_force);
    }
  }

  //DiagnosticTimer timer("Sensor[" + this->GetName() + "] Update");

  //Time physics_dt = this->GetWorld()->GetPhysicsEngine()->GetStepTime();

  //if (((this->GetWorld()->GetSimTime() - this->lastUpdate - this->updatePeriod)/physics_dt) >= 0)
  {
    //this->lastUpdate = this->GetWorld()->GetSimTime();
  }

  // update any controllers that are children of sensors, e.g. ros_bumper
  //if (this->controller)
    //this->controller->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the sensor
void Sensor::Fini()
{
  this->plugins.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Get name 
std::string Sensor::GetName() const
{
  return this->sdf->GetValueString("name");
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Sensor::LoadPlugin( sdf::ElementPtr &_sdf )
{
  std::string name = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  //gzerr << "Sensor LoadPlugin [" << name << "] [" << filename << "]\n";
  gazebo::SensorPluginPtr plugin = gazebo::SensorPlugin::Create(filename, name);

  if (plugin)
  {
    SensorPtr myself = shared_from_this();
    plugin->Load(myself, _sdf);
    this->plugins.push_back( plugin );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set whether the sensor is active or not
void Sensor::SetActive(bool value)
{
  this->active = value;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set whether the sensor is active or not
bool Sensor::IsActive()
{
  return this->active;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the current pose
math::Pose Sensor::GetPose() const
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the update Hz rate
void Sensor::SetUpdateRate(double _hz)
{
  if (_hz > 0.0)
    this->updatePeriod = 1.0/_hz;
  else
    this->updatePeriod = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
// return last render time
common::Time Sensor::GetLastUpdateTime()
{
  return this->lastUpdateTime;
}


