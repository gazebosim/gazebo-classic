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
 * Desc: A generic ptz controller
 * Author: Nathan Koenig
 * Date: 26 Nov 2007
 * SVN: $Id$
 */

#include <algorithm>
#include <assert.h>

#include "World.hh"
#include "Model.hh"
#include "common/Global.hh"
#include "common/XMLConfig.hh"
#include "Simulator.hh"
#include "common/GazeboError.hh"
#include "ControllerFactory.hh"
#include "Joint.hh"
#include "Generic_PTZ.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("generic_ptz", Generic_PTZ);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Generic_PTZ::Generic_PTZ(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Generic_PTZ controller requires a Model as its parent");

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  Param::Begin(&this->parameters);
  this->panJointNameP = new ParamT<std::string>("panJoint", "", 1);
  this->tiltJointNameP = new ParamT<std::string>("tiltJoint", "", 1);
  this->motionGainP = new ParamT<double>("motionGain",2,0);
  this->forceP = new ParamT<double>("force",0.01,0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Generic_PTZ::~Generic_PTZ()
{
  //if (this->panJoint)
  //  delete this->panJoint;
  //if (this->tiltJoint)
  //  delete this->tiltJoint;

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  delete this->panJointNameP;
  delete this->tiltJointNameP;
  delete this->motionGainP;
  delete this->forceP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Generic_PTZ::LoadChild(XMLConfigNode *node)
{
  this->ptzIface = dynamic_cast<libgazebo::PTZIface*>(this->GetIface("ptz"));

  this->panJointNameP->Load(node);
  this->tiltJointNameP->Load(node);
  this->motionGainP->Load(node);
  this->forceP->Load(node);

  this->panJoint = this->myParent->GetJoint(**this->panJointNameP);
  this->tiltJoint = this->myParent->GetJoint(**this->tiltJointNameP);

  if (!this->panJoint)
    gzthrow("couldn't get pan hinge joint");

  if (!this->tiltJoint)
    gzthrow("couldn't get tilt hinge joint");

}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void Generic_PTZ::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->panJointNameP) << "\n";
  stream << prefix << *(this->tiltJointNameP) << "\n";
  stream << prefix << *(this->motionGainP) << "\n";
  stream << prefix << *(this->forceP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Generic_PTZ::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void Generic_PTZ::ResetChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Generic_PTZ::UpdateChild()
{
  float tiltSpeed = 0;  
  float panSpeed = 0; 

  this->ptzIface->Lock(1);
  if (this->ptzIface->data->control_mode == GAZEBO_PTZ_POSITION_CONTROL)
  {
    this->cmdPan = this->ptzIface->data->cmd_pan;
    this->cmdTilt = this->ptzIface->data->cmd_tilt;
    //this->cmdZoom = this->hfov / this->ptzIface->data->cmd_zoom;

    // Apply joint limits to commanded pan/tilt angles
    if (this->cmdTilt > M_PI*0.3)
      this->cmdTilt = M_PI*0.3;
    else if (this->cmdTilt < -M_PI*0.3)
      this->cmdTilt = -M_PI*0.3;

    if (this->cmdPan > M_PI*0.3)
      this->cmdPan = M_PI*0.3;
    else if (this->cmdPan < -M_PI*0.3)
      this->cmdPan = -M_PI*0.3;

    // Apply limits on commanded zoom
    //if (this->cmdZoom < this->zoomMin)
    // this->cmdZoom = this->zoomMin;
    //if (this->cmdZoom > this->zoomMax)
    // this->cmdZoom = this->zoomMax;

    // Set the pan and tilt motors; can't set angles so track cmds with
    // a proportional control
    tiltSpeed = this->cmdTilt - this->tiltJoint->GetAngle(0).GetAsRadian();
    panSpeed = this->cmdPan - this->panJoint->GetAngle(0).GetAsRadian();
  }
  else
  {
    tiltSpeed = this->ptzIface->data->cmd_tilt_speed;
    panSpeed = this->ptzIface->data->cmd_pan_speed;
  }

  this->ptzIface->Unlock();

  if (fabs(tiltSpeed) > 0.01)
    this->tiltJoint->SetVelocity( 0, **(this->motionGainP) * tiltSpeed);
  else
    this->tiltJoint->SetVelocity( 0, 0);

  this->tiltJoint->SetMaxForce( 0, **(this->forceP) );

  if (fabs(panSpeed) > 0.01)
    this->panJoint->SetVelocity( 0, **(this->motionGainP) * panSpeed);
  else
    this->panJoint->SetVelocity( 0, 0);

  this->panJoint->SetMaxForce( 0, **(this->forceP) );

  this->PutPTZData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Generic_PTZ::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void Generic_PTZ::PutPTZData()
{
  libgazebo::PTZData *data = this->ptzIface->data;

  this->ptzIface->Lock(1);

  // Data timestamp
  data->head.time = this->myParent->GetWorld()->GetSimTime().Double();

  data->pan = this->panJoint->GetAngle(0).GetAsRadian();
  data->tilt = this->tiltJoint->GetAngle(0).GetAsRadian();
  //data->zoom = this->camera->GetFOV();

  this->ptzIface->Unlock();

  // New data is available
  this->ptzIface->Post();
}

