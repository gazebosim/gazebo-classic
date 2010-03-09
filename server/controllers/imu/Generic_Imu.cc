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
 * Desc:  Generic IMU controller.
 * Author: Matt Thompson
 * Date: 07 September 2008
 * SVN info: $Id$
 */

#include <algorithm>
#include <assert.h>

#include "Global.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Generic_Imu.hh"
#include "ImuSensor.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("generic_imu", Generic_Imu);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Generic_Imu::Generic_Imu(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<ImuSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GenericImu controller requires an IMU Sensor as its parent");

  this->imuIface = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Generic_Imu::~Generic_Imu()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Generic_Imu::LoadChild(XMLConfigNode *node)
{
  this->imuIface = dynamic_cast<ImuIface*>(this->GetIface("imu"));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Generic_Imu::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Generic_Imu::UpdateChild()
{
  bool imuOpened = false;

  if (this->imuIface->Lock(1))
  {
    imuOpened = this->imuIface->GetOpenCount() > 0;
    this->imuIface->Unlock();
  }

  if (imuOpened)
  {
    this->myParent->SetActive(true);
    this->PutImuData();
  }

  if (!imuOpened)
  {
    this->myParent->SetActive(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Generic_Imu::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put IMU data to the interface
void Generic_Imu::PutImuData()
{

  Pose3d velocity;
  velocity = this->myParent->GetVelocity();

  if (this->imuIface->Lock(1))
  {
    // Data timestamp
    //this->imuIface->data->head.time = Simulator::Instance()->GetSimTime();
    
    this->imuIface->data->velocity.pos.x = velocity.pos.x;
    this->imuIface->data->velocity.pos.y = velocity.pos.y;
    this->imuIface->data->velocity.pos.z = velocity.pos.z;
    this->imuIface->data->velocity.roll = velocity.rot.x;
    this->imuIface->data->velocity.pitch = velocity.rot.y;
    this->imuIface->data->velocity.yaw = velocity.rot.z;

    this->imuIface->Unlock();

    // New data is available
    this->imuIface->Post();
  }
}
