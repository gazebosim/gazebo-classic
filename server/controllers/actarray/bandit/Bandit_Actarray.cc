/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Actuator array controller for a Bandit robot.
 * Author: Nathan Koenig
 * Date: 19 Sep 2007
 * SVN info: $Id$
 */

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "HingeJoint.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "GazeboError.hh"
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
    GZ_DELETE(this->jointNamesP[i]);
    GZ_DELETE(this->forcesP[i]);
    GZ_DELETE(this->gainsP[i]);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Bandit_Actarray::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *jNode;
  int i =0;
  this->myIface = dynamic_cast<ActarrayIface*>(this->ifaces[0]);

  if (!this->myIface)
    gzthrow("Bandit_Actarray controller requires a Actarray Iface");


  Param::Begin(&this->parameters);
  for (i=0, jNode = node->GetChild("joint"); jNode; i++)
  {
    this->jointNamesP[i] = new ParamT<std::string>("name","",1);
    this->jointNamesP[i]->Load(jNode);

    this->forcesP[i] = new ParamT<float>("force",0.0,1);
    this->forcesP[i]->Load(jNode);

    this->gainsP[i] = new ParamT<float>("gain",0.0,1);
    this->gainsP[i]->Load(jNode);
   
    this->joints[i] = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->jointNamesP[i]->GetValue()));

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
    this->joints[i]->SetParam( dParamVel, 0.0);
    this->joints[i]->SetParam( dParamFMax, **(this->forcesP[i]) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Bandit_Actarray::UpdateChild()
{
  HingeJoint *joint = NULL;
  float angle;

  this->myIface->Lock(1);
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime();

  this->myIface->data->actuators_count = JOINTCNT;

  for (unsigned int i=0; i<JOINTCNT; i++)
  {
    double cmdAngle = this->myIface->data->cmd_pos[i];
    double cmdSpeed = this->myIface->data->cmd_speed[i];

    joint = dynamic_cast<HingeJoint*>(this->joints[i]);

    if (this->myIface->data->joint_mode[i] == GAZEBO_ACTARRAY_JOINT_POSITION_MODE)
    {
      if (cmdAngle > joint->GetHighStop())
      {
        cmdAngle = joint->GetHighStop();
      }
      else if (cmdAngle < joint->GetLowStop())
      {
        cmdAngle = joint->GetLowStop();
      }

      angle = cmdAngle - joint->GetAngle();

      if (fabs(angle) > 0.01)
      {
        joint->SetParam( dParamVel, **(this->gainsP[i]) * angle);
        joint->SetParam( dParamFMax, **(this->forcesP[i]) );
      }

    }
    else if (this->myIface->data->joint_mode[i] == GAZEBO_ACTARRAY_JOINT_SPEED_MODE)
    {
      joint->SetParam( dParamVel, cmdSpeed );
      joint->SetParam( dParamFMax, **(this->forcesP[i]) );
    }


    this->myIface->data->actuators[i].position = joint->GetAngle();
    this->myIface->data->actuators[i].speed = joint->GetAngleRate();
  }

  this->myIface->data->new_cmd = 0;

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Bandit_Actarray::FiniChild()
{
}
