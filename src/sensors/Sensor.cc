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

#include "common/Timer.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/XMLConfig.hh"

#include "sensors/Sensor.hh"

using namespace gazebo;
using namespace sensors;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor()
{
  this->active = true;

  common::Param::Begin(&this->parameters);
  this->nameP = new common::ParamT<std::string>("name", "sensor_default_name",0);
  this->updateRateP = new common::ParamT<double>("update_rate", 0, 0);
  this->alwaysActiveP = new common::ParamT<bool>("always_active", false, 0);
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
  delete this->nameP;
  delete this->updateRateP;
  delete this->alwaysActiveP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load(common::XMLConfigNode *node)
{
  this->nameP->Load(node);
  this->updateRateP->Load(node);
  this->alwaysActiveP->Load(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void Sensor::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";

  stream << prefix << "<sensor:" << this->typeName << " name=\"" << this->nameP->GetValue() << "\">\n";

  stream << prefix << *(this->updateRateP) << "\n";

  stream << prefix << "</sensor:" << this->typeName << ">\n";
}
 
////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update(bool /*force_*/)
{
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
  //if (this->controller)
    //this->controller->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Get name 
std::string Sensor::GetName() const
{
  return **this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Sensor::LoadController(common::XMLConfigNode * /*node_*/)
{
  /*
  if (!node)
  {
    gzwarn << this->GetName() << " sensor has no controller.\n";
    return;
  }


  //Iface *iface;
  //XMLConfigNode *childNode;
  std::ostringstream stream;

  // Get the controller's type
  std::string controllerType = node->GetName();

  // Get the unique name of the controller
  std::string controllerName = node->GetString("name","",1);

  // Create the interface
  //if ( (childNode = node->GetChildByNSPrefix("interface")) )
  //{
  //  // Get the type of the interface (eg: laser)
  //  std::string ifaceType = childNode->GetName();

  //  // Get the name of the iface
  //  std::string ifaceName = childNode->GetString("name","",1);

  //  // Use the factory to get a new iface based on the type
  //  iface = IfaceFactory::NewIface(ifaceType);

  //  // Create the iface
  //  iface->Create(this->GetWorld()->GetGzServer(), ifaceName);

  //}
  //else
  //{
  //  stream << "No interface defined for " << controllerName << "controller";
  //  gzthrow(stream.str());
  //}
  
  // See if the controller is in a plugin
  std::string pluginName = node->GetString("plugin","",0);
  if (pluginName != "")
    ControllerFactory::LoadPlugin(pluginName, controllerType);

  // Create the controller based on it's type
  this->controller = ControllerFactory::NewController(controllerType, this);

  // Load the controller if it's available
  if (this->controller) this->controller->Load(node);
  */
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
