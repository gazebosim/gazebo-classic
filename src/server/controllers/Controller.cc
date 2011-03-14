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
/*
 * Desc: Controller base class.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "Timer.hh"
#include "Model.hh"
#include "Sensor.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "Controller.hh"
#include "Simulator.hh"
#include "PhysicsEngine.hh"
#include "Global.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Controller::Controller(Entity *entity )
{
  Param::Begin(&this->parameters);
  this->typeP = new ParamT<std::string>("type","",1);
  this->nameP = new ParamT<std::string>("name","",1);
  this->alwaysOnP = new ParamT<bool>("always_on", false, 0);
  this->updatePeriodP = new ParamT<double>("update_rate", 10, 0);
  Param::End();

  if (!dynamic_cast<Model*>(entity) && !dynamic_cast<Sensor*>(entity))
  {
    gzthrow("The parent of a controller must be a Model or a Sensor");
  }

  this->parent = entity;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Controller::~Controller()
{
  delete this->typeP;
  delete this->nameP;
  delete this->alwaysOnP;
  delete this->updatePeriodP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the controller. Called once on startup
void Controller::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode;

  if (!this->parent)
    gzthrow("Parent entity has not been set");

  this->typeP->Load(node);
  this->nameP->Load(node);

  this->alwaysOnP->Load(node);

  this->updatePeriodP->Load(node);

  this->lastUpdate = this->parent->GetWorld()->GetSimTime();

  childNode = node->GetChild("interface");
  
  // Create the interfaces
  while (childNode)
  {
    libgazebo::Iface *iface=0;

    // Get the type of the interface (eg: laser)
    std::string ifaceType = childNode->GetString("type","",1);

    // Get the name of the iface
    std::string ifaceName = childNode->GetString("name","",1);

    // Constructor the heirarchical name for the iface
    Common *p = parent;
    while (p != NULL)
    {
      Model *m = dynamic_cast<Model*>(p);
      if (m)
        ifaceName.insert(0, m->GetName()+"::");
      p = p->GetParent();
    }

    try
    {
      // Use the factory to get a new iface based on the type
      iface = libgazebo::IfaceFactory::NewIface(ifaceType);
    }
    catch (...) //TODO: Show the exception text here (subclass exception?)
    {
      gzmsg(1) << "No libgazebo Iface for the interface[" << ifaceType << "] found. Disabled.\n";
      childNode = childNode->GetNext("interface");
      continue;
    }
    
    // Create the iface
    try
    {
      iface->Create(this->parent->GetWorld()->GetGzServer(), ifaceName);
    }
    catch (std::string e)
    {
      gzthrow(e);
    }
    
    this->ifaces.push_back(iface);

    childNode = childNode->GetNext("interface");
  }

  this->LoadChild(node);
}


////////////////////////////////////////////////////////////////////////////////
/// Initialize the controller. Called once on startup.
void Controller::Init()
{
  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Controller::Save(std::string &prefix, std::ostream &stream)
{
  std::vector<libgazebo::Iface*>::iterator iter;

  stream << prefix << "<controller type=\"" << **this->typeP << "\" name=\"" << **this->nameP << "\">\n";

  stream << prefix << "  " << *(this->updatePeriodP) << "\n";

  // Ouptut the interfaces
  for (iter = this->ifaces.begin(); iter != this->ifaces.end(); iter++)
  {
    stream << prefix << "  <interface type=\"" << (*iter)->GetType() << "\" name=\"" << (*iter)->GetId() << "\"/>\n";
  }

  std::string p = prefix + "  ";

  this->SaveChild(p, stream);

  stream << prefix << "</controller>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void Controller::Reset()
{
  this->ResetChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the controller. Called every cycle.
void Controller::Update()
{
  if (this->IsConnected() || **this->alwaysOnP)
  {
    //DiagnosticTimer timer("Controller[" + this->GetName() +"] Update Timer");

    // round time difference to this->physicsEngine->GetStepTime()
    Time physics_dt = this->parent->GetWorld()->GetPhysicsEngine()->GetStepTime();

    //timer.Start();

    Time simTime = this->parent->GetWorld()->GetSimTime();
    if ((simTime-lastUpdate - **this->updatePeriodP)/physics_dt >= 0)
    {
      this->UpdateChild();
      lastUpdate = this->parent->GetWorld()->GetSimTime();
      //timer.Report("Update() dt");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Finialize the controller. Called once on completion.
void Controller::Fini()
{
  std::vector<libgazebo::Iface*>::iterator iter;

  for (iter=this->ifaces.begin(); iter!=this->ifaces.end(); iter++)
  {
    delete *iter;
  }
  this->ifaces.clear();

  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if an interface is open 
bool Controller::IsConnected() const
{
  std::vector<libgazebo::Iface*>::const_iterator iter;

  // if the alwaysOn flag is true, this controller is connected
  if (this->alwaysOnP->GetValue())
    return true;

  for (iter=this->ifaces.begin(); iter!=this->ifaces.end(); iter++)
  {
    if ((*iter)->GetOpenCount() > 0)
      return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Get the name of the controller
std::string Controller::GetName() const
{
  return this->nameP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
// Get a interface of the controller
libgazebo::Iface* Controller::GetIface(std::string type, bool mandatory, int number)
{
  std::vector<libgazebo::Iface*>::iterator iter;
  int order = number;
  libgazebo::Iface *iface = NULL;

  for (iter = this->ifaces.begin(); iter != this->ifaces.end(); iter++)
  {
    if ((*iter)->GetType() == type)
    {
      if (order == 0)
        iface = (*iter);
      else
        order --;
    }
  }
  
  if ((iface == NULL) and mandatory)
  {
    std::ostringstream stream;
    stream << "Controller " << this->GetName() << "trying to get " << type << " interface but it is not defined";
    gzthrow(stream.str());
  }
  return iface;
}

void Controller::GetInterfaceNames(std::vector<std::string>& list) const{
  
  std::vector<libgazebo::Iface*>::const_iterator iter;
  
  for (iter = this->ifaces.begin(); iter != this->ifaces.end(); iter++)
  {
    std::string str;
    str=(*iter)->GetId()+">>"+(*iter)->GetType();
    list.push_back(str);
    
  }


}  
