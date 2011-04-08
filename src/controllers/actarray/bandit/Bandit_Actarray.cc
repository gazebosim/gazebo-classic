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
 * Desc: Actuator array controller for a Bandit robot.
 * Author: Nathan Koenig
 * Date: 19 Sep 2007
 * SVN info: $Id$
 */

#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Model.hh"
#include "Joint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "common/Exception.hh"
#include "ControllerFactory.hh"
#include "Bandit_Actarray.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("bandit_actarray", Bandit_Actarray);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Bandit_Actarray::Bandit_Actarray(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Bandit_Actarray controller requires a Model as its parent");

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Bandit_Actarray::~Bandit_Actarray()
{
  for (int i=0; i<JOINTCNT; i++)
  {
    if (this->jointNamesP[i])
      delete this->jointNamesP[i];
    this->jointNamesP[i] = NULL;
      
    if(this->forcesP[i])
      delete this->forcesP[i];
    this->forcesP[i] = NULL;

    if(this->gainsP[i])
      delete this->gainsP[i];
    this->gainsP[i] = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Bandit_Actarray::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *jNode;
  int i =0;
  this->myIface = dynamic_cast<libgazebo::ActarrayIface*>(this->GetIface("actarray"));

  Param::Begin(&this->parameters);
  for (i=0, jNode = node->GetChild("joint"); jNode; i++)
  {
    this->jointNamesP[i] = new ParamT<std::string>("name","",1);
    this->jointNamesP[i]->Load(jNode);

    this->forcesP[i] = new ParamT<float>("force",0.0,1);
    this->forcesP[i]->Load(jNode);

    this->gainsP[i] = new ParamT<float>("gain",0.0,1);
    this->gainsP[i]->Load(jNode);
   
    this->joints[i] = this->myParent->GetJoint(**this->jointNamesP[i]);

    jNode = jNode->GetNext("joint");
  }
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Bandit_Actarray::SaveChild(std::string &prefix, std::ostream &stream)
{
  for (int i=0; i<JOINTCNT; i++)
  {
    stream << prefix << "<joint name=\"" << this->jointNamesP[i]->GetValue() << "\">\n";
    stream << prefix << "  " << *(this->forcesP[i]) << "\n";
    stream << prefix << "  " << *(this->gainsP[i]) << "\n";
    stream << prefix << "</joint>\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Bandit_Actarray::InitChild()
{
  for (int i=0; i<JOINTCNT; i++)
  {
    this->joints[i]->SetVelocity(0, 0.0);
    this->joints[i]->SetMaxForce(0, **(this->forcesP[i]) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Bandit_Actarray::UpdateChild()
{
  Joint *joint = NULL;
  Angle angle;

  this->myIface->Lock(1);
  this->myIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  this->myIface->data->actuators_count = JOINTCNT;

  for (unsigned int i=0; i<JOINTCNT; i++)
  {
    Angle cmdAngle = this->myIface->data->cmd_pos[i];
    double cmdSpeed = this->myIface->data->cmd_speed[i];

    joint = this->joints[i];

    if (this->myIface->data->joint_mode[i] == GAZEBO_ACTARRAY_JOINT_POSITION_MODE)
    {
      if (cmdAngle > joint->GetHighStop(0))
      {
        cmdAngle = joint->GetHighStop(0);
      }
      else if (cmdAngle < joint->GetLowStop(0))
      {
        cmdAngle = joint->GetLowStop(0);
      }

      angle = cmdAngle - joint->GetAngle(0);

      if (fabs(angle.GetAsRadian()) > 0.01)
        joint->SetVelocity(0, **(this->gainsP[i]) * angle.GetAsRadian());
      else
        joint->SetVelocity(0, 0);

      joint->SetMaxForce( 0, **(this->forcesP[i]) );

    }
    else if (this->myIface->data->joint_mode[i] == GAZEBO_ACTARRAY_JOINT_SPEED_MODE)
    {
      joint->SetVelocity( 0, cmdSpeed );
      joint->SetMaxForce( 0, **(this->forcesP[i]) );
    }

    this->myIface->data->actuators[i].position = 
      joint->GetAngle(0).GetAsRadian();
    this->myIface->data->actuators[i].speed = joint->GetVelocity(0);
  }

  this->myIface->data->new_cmd = 0;

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Bandit_Actarray::FiniChild()
{
}
