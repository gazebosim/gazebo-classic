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
  this->imuIface = dynamic_cast<libgazebo::ImuIface*>(this->GetIface("imu"));
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
    //this->imuIface->data->head.time = this->myParent->GetWorld()->GetSimTime();
    
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
