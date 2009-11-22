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
 * Desc: IR array controller.
 * Author: Wenguo Liu
 * Date: 24 Apr 2008
 */

#include <algorithm>
#include <assert.h>

#include "Sensor.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "IRSensor.hh"
#include "IR_Array.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("irarray", IR_Array);

////////////////////////////////////////////////////////////////////////////////
// Constructor
IR_Array::IR_Array(Entity *parent)
    : Controller(parent)
{
   this->myParent = dynamic_cast<IRSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("IR_Array controller requires a IRSensor as its parent");

  this->irIface = NULL;
  
  
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
IR_Array::~IR_Array()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void IR_Array::LoadChild(XMLConfigNode *node)
{
  this->irIface = dynamic_cast<IRIface*>(this->GetIface("irarray"));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void IR_Array::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void IR_Array::UpdateChild()
{
  /*bool opened = false;

  if (this->irIface->Lock(1))
  {
    opened = this->irIface->GetOpenCount() > 0;
    this->irIface->Unlock();
  }
*/
  //if (opened)
  {
    this->PutIRData();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void IR_Array::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void IR_Array::PutIRData()
{
  if (this->irIface->Lock(1))
  {
    // Data timestamp
    this->irIface->data->head.time = Simulator::Instance()->GetSimTime().Double();
    
    this->irIface->data->ir_count = this->myParent->GetIRCount();
    
    this->irIface->data->range_count = this->myParent->GetIRCount();
    
    for(int i=0;i<this->irIface->data->ir_count;i++)
    {
       this->irIface->data->ranges[i] = this->myParent->GetRange(i);
       Pose3d pose = this->myParent->GetPose(i);

       this->irIface->data->poses[i].pos.x = pose.pos.x;
       this->irIface->data->poses[i].pos.y = pose.pos.y;
       this->irIface->data->poses[i].pos.z = pose.pos.z;

       this->irIface->data->poses[i].roll = pose.rot.GetRoll();
       this->irIface->data->poses[i].pitch = pose.rot.GetPitch();
       this->irIface->data->poses[i].yaw = pose.rot.GetYaw();
    }

    this->irIface->Unlock();

    // New data is available
    this->irIface->Post();
  }
}


