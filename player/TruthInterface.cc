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
/* Desc: Truth Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "TruthInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
TruthInterface::TruthInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Truth Interface
  this->iface = gz_truth_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
TruthInterface::~TruthInterface()
{
  // Release this interface
  gz_truth_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int TruthInterface::ProcessMessage(QueuePointer &respQueue,
                   player_msghdr_t *hdr, void *data)
{
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_TRUTH_REQ_SET_POSE, this->device_addr))
  {
    double q[4];
    double e[3];

    player_truth_pose_t *req = (player_truth_pose_t*) data;

    gz_truth_lock(this->iface, 1);

    this->iface->data->cmd_pos[0] = req->pose.px;
    this->iface->data->cmd_pos[1] = req->pose.py;
    this->iface->data->cmd_pos[2] = req->pose.pz;

    e[0] = req->pose.proll;
    e[1] = req->pose.ppitch;
    e[2] = req->pose.pyaw;

    gz_truth_quatern_from_euler(this->iface, q,e);

    this->iface->data->cmd_rot[0] = q[0];
    this->iface->data->cmd_rot[1] = q[1];
    this->iface->data->cmd_rot[2] = q[2];
    this->iface->data->cmd_rot[3] = q[3];

    this->iface->data->cmd_new = 1;

    gz_truth_unlock(this->iface);

    this->driver->Publish(this->device_addr, respQueue, 
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_TRUTH_REQ_SET_POSE,
        &req, sizeof(req), NULL);

    return 0;
  }
 
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void TruthInterface::Update()
{
  player_truth_pose_t data;
  double e[3];
  struct timeval ts;

  gz_truth_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    data.pose.px = this->iface->data->pos[0];
    data.pose.py = this->iface->data->pos[1];
    data.pose.pz = this->iface->data->pos[2];

    // Convert the rotation from quaternion to euler 
    gz_truth_euler_from_quatern(this->iface, e, this->iface->data->rot);

    data.pose.proll = e[0];
    data.pose.ppitch = e[1];
    data.pose.pyaw = e[2];
    
    this->driver->Publish( this->device_addr, NULL,
        PLAYER_MSGTYPE_DATA, PLAYER_TRUTH_DATA_POSE, 
        (void*)&data, sizeof(data), &this->datatime );
  }

  gz_truth_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void TruthInterface::Subscribe()
{
  // Open the interface
  if (gz_truth_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void TruthInterface::Unsubscribe()
{
  gz_truth_close(this->iface);
}
