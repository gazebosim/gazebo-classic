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
 * Desc: Actuator array controller for a Carl robot.
 * Author: Nathan Koenig
 * Date: 19 Sep 2007
 * SVN info: $Id$
 */

#include "Global.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "Joint.hh"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Generic_Actarray.hh"
#include <typeinfo>
#include <iostream>

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("generic_actarray", Generic_Actarray);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Generic_Actarray::Generic_Actarray(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Generic_Actarray controller requires a Model as its parent");

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Generic_Actarray::~Generic_Actarray()
{
  delete [] joints;
  //joints = NULL;
  delete [] forces;
  forces = NULL;
  delete [] gains;
  gains = NULL;
  delete [] tolerances;
  tolerances = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Generic_Actarray::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *jNode;
  this->myIface = dynamic_cast<libgazebo::ActarrayIface*>(this->GetIface("actarray"));

  for (n_joints=0, jNode = node->GetChild("joint"); jNode; n_joints++, jNode = jNode->GetNext("joint"))
  {
  }

  this->joints     = new Joint*[n_joints];
  this->forces     = new float[n_joints];
  this->gains      = new float[n_joints];
  this->tolerances = new float[n_joints];
  int i;
  for (i=0, jNode = node->GetChild("joint"); jNode; i++)
  {
    std::string name = jNode->GetString("name","",1);

    this->joints[i]     = this->myParent->GetJoint(name);
    this->forces[i]     = jNode->GetDouble("force",0.0,1);
    this->gains[i]      = jNode->GetDouble("gain",0.0,1);
    this->tolerances[i] = jNode->GetDouble("tolerance",0.01,0);

    jNode = jNode->GetNext("joint");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Generic_Actarray::InitChild()
{
    for (int i=0; i<n_joints; i++)
  {
    this->joints[i]->SetVelocity( 0, 0.0);
    this->joints[i]->SetMaxForce( 0, this->forces[i] );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Generic_Actarray::UpdateChild()
{
  Joint *joint;
  Angle delta_position;
  Angle actual_position;
  float actual_speed;

  this->myIface->Lock(1);
  this->myIface->data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  this->myIface->data->actuators_count = n_joints;

  for (int i=0; i<n_joints; i++)
  {
    Angle target_position( this->myIface->data->cmd_pos[i]);
    joint = this->joints[i];

    if (target_position > joint->GetHighStop(0))
    {
      target_position = joint->GetHighStop(0);
    }
    else if (target_position < joint->GetLowStop(0))
    {
      target_position = joint->GetLowStop(0);
    }

    actual_position = joint->GetAngle(0);
    actual_speed    = joint->GetVelocity(0);

    delta_position = target_position - actual_position;

    if (fabs(delta_position.GetAsRadian()) > tolerances[i])
      joint->SetVelocity( 0,  this->gains[i] * delta_position.GetAsRadian());
    else
      joint->SetVelocity( 0,  0);

    joint->SetMaxForce( 0, this->forces[i] );

    this->myIface->data->actuators[i].position = actual_position.GetAsRadian();
    this->myIface->data->actuators[i].speed = actual_speed;
  }

  this->myIface->data->new_cmd = 0;

  this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Generic_Actarray::FiniChild()
{
}
