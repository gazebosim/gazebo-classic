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
 * SVN: $Id$
 */

#include "common/Timer.hh"
#include "Controller.hh"
#include "common/GazeboMessage.hh"
#include "common/GazeboError.hh"
#include "Body.hh"
#include "common/XMLConfig.hh"
#include "World.hh"
#include "ControllerFactory.hh"
#include "Simulator.hh"
#include "Sensor.hh"
#include "Simulator.hh"
#include "PhysicsEngine.hh"
#include "common/Global.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor(Body *body)
    : Entity(body)
{
  this->body = body;
  this->controller = NULL;
  this->active = true;

  this->lastUpdate = this->GetWorld()->GetSimTime();

  Param::Begin(&this->parameters);
  this->updateRateP = new ParamT<double>("update_rate", 0, 0);
  this->alwaysActiveP = new ParamT<bool>("always_active", false, 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
  delete this->updateRateP;
  delete this->alwaysActiveP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load(XMLConfigNode *node)
{
  this->nameP->Load(node);
  this->updateRateP->Load(node);
  this->alwaysActiveP->Load(node);

  if (**(this->updateRateP) == 0)
    this->updatePeriod = 0.0;
  else
    this->updatePeriod = 1.0 / **(updateRateP);

  this->LoadController( node->GetChild("controller") );
  this->LoadChild(node);

  double updateRate  = node->GetDouble("updateRate", 0, 0);
  if (updateRate == 0)
    this->updatePeriod = 0.0; // no throttling if updateRate is 0
  else
    this->updatePeriod = 1.0 / updateRate;
  this->lastUpdate = this->GetWorld()->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void Sensor::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";

  stream << prefix << "<sensor:" << this->typeName << " name=\"" << this->nameP->GetValue() << "\">\n";

  stream << prefix << *(this->updateRateP) << "\n";

  this->SaveChild(prefix, stream);

  if (this->controller)
    this->controller->Save(p, stream);

  stream << prefix << "</sensor:" << this->typeName << ">\n";
}
 
////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
  if (this->controller)
    this->controller->Init();

  this->lastUpdate = this->GetWorld()->GetSimTime();

  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update()
{
  //DiagnosticTimer timer("Sensor[" + this->GetName() + "] Update");

  Time physics_dt = this->GetWorld()->GetPhysicsEngine()->GetStepTime();

  if (((this->GetWorld()->GetSimTime() - this->lastUpdate - this->updatePeriod)/physics_dt) >= 0)
  {
    this->UpdateChild();
    this->lastUpdate = this->GetWorld()->GetSimTime();
  }

  // update any controllers that are children of sensors, e.g. ros_bumper
  if (this->controller)
    this->controller->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the sensor
void Sensor::Fini()
{
  if (this->controller)
    this->controller->Fini();
  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Sensor::LoadController(XMLConfigNode *node)
{
  if (!node)
  {
    gzmsg(0) << this->GetName() << " sensor has no controller.\n";
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
  /*if ( (childNode = node->GetChildByNSPrefix("interface")) )
  {
    // Get the type of the interface (eg: laser)
    std::string ifaceType = childNode->GetName();

    // Get the name of the iface
    std::string ifaceName = childNode->GetString("name","",1);

    // Use the factory to get a new iface based on the type
    iface = IfaceFactory::NewIface(ifaceType);

    // Create the iface
    iface->Create(this->GetWorld()->GetGzServer(), ifaceName);

  }
  else
  {
    stream << "No interface defined for " << controllerName << "controller";
    gzthrow(stream.str());
  }*/
  
  // See if the controller is in a plugin
  std::string pluginName = node->GetString("plugin","",0);
  if (pluginName != "")
    ControllerFactory::LoadPlugin(pluginName, controllerType);

  // Create the controller based on it's type
  this->controller = ControllerFactory::NewController(controllerType, this);

  // Load the controller if it's available
  if (this->controller) this->controller->Load(node);
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
Pose3d Sensor::GetPose() const
{
  return this->body->GetWorldPose();
}

///////////////////////////////////////////////////////////////////////////////
/// Get the name of the interfaces define in the sensor controller
void Sensor::GetInterfaceNames(std::vector<std::string>& list) const
{
  controller->GetInterfaceNames(list);
}

///////////////////////////////////////////////////////////////////////////////
/// Get the time of the last update
Time Sensor::GetLastUpdate()
{
  return this->lastUpdate;
}


