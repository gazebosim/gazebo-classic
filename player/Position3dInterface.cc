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
/* Desc: Position 3d Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Postion3d Interface
- PLAYER_POSITION3D_CMD_SET_VEL
- PLAYER_POSITION3D_SET_ODOM
- PLAYER_POSITION3D_MOTOR_POWER
*/
/* TODO
PLAYER_POSITION3D_GET_GEOM
PLAYER_POSITION3D_RESET_ODOM
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "Position3dInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
Position3dInterface::Position3dInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_position_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
Position3dInterface::~Position3dInterface()
{
  // Release this interface
  gz_position_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int Position3dInterface::ProcessMessage(MessageQueue *respQueue,
                   player_msghdr_t *hdr, void *data)
{
  gz_position_lock(this->iface, 1);

  // COMMAND VELOCITY:
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
        PLAYER_POSITION3D_CMD_SET_VEL, this->device_addr))
  {
    player_position3d_cmd_vel_t *cmd;

    cmd = (player_position3d_cmd_vel_t*) data;

    this->iface->data->cmd_vel_pos[0] = cmd->vel.px;
    this->iface->data->cmd_vel_pos[1] = cmd->vel.py;
    this->iface->data->cmd_vel_pos[2] = cmd->vel.pz;
    this->iface->data->cmd_vel_rot[0] = cmd->vel.proll;
    this->iface->data->cmd_vel_rot[1] = cmd->vel.ppitch;
    this->iface->data->cmd_vel_rot[2] = cmd->vel.pyaw;

    return 0;
  }

  // REQUEST SET ODOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_POSITION3D_SET_ODOM, this->device_addr))
  {
    if (hdr->size != sizeof(player_position3d_set_odom_req_t))
    {
      PLAYER_WARN("Arg to odometry set requestes wrong size; ignoring");
      return -1;
    }

    player_position3d_set_odom_req_t *odom = (player_position3d_set_odom_req_t*)data;

    this->iface->data->pos[0] = odom->pos.px;
    this->iface->data->pos[1] = odom->pos.py;
    this->iface->data->pos[2] = odom->pos.pz;

    this->iface->data->rot[0] = odom->pos.proll;
    this->iface->data->rot[1] = odom->pos.ppitch;
    this->iface->data->rot[2] = odom->pos.pyaw;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_SET_ODOM);

    return 0;
  }

  // REQUEST SET MOTOR POWER
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
        PLAYER_POSITION3D_MOTOR_POWER, this->device_addr))
  {
    if (hdr->size != sizeof(player_position3d_power_config_t))
    {
      PLAYER_WARN("Arg to motor set requestes wrong size; ignoring");
      return -1;
    }

    player_position3d_power_config_t *power;

    power = (player_position3d_power_config_t*) data;

    this->iface->data->cmd_enable_motors = power->state;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_MOTOR_POWER);

    return 0;
  }

  // REQUEST GET GEOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_POSITION3D_GET_GEOM, this->device_addr))
  {
    if (hdr->size != 0)
    {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      return -1;
    }


    player_position3d_geom_t geom;

    // TODO: get correct dimensions; there are for the P2AT

    geom.pose.px = 0;
    geom.pose.py = 0;
    geom.pose.pz = 0;

    geom.size.sw= 0.53;
    geom.size.sl = 0.38;
    geom.size.sh = 0.2;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_GET_GEOM, 
        &geom, sizeof(geom), NULL);

    return 0;
  }

  // REQUEST RESET ODOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_POSITION3D_RESET_ODOM, this->device_addr))
  {
    if (hdr->size != 0)
    {
      PLAYER_WARN("Arg reset position request is wrong size; ignoring");
      return -1;
    }

    // TODO: Make this work!!
    //
    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_RESET_ODOM);

    return 0;
  }

  gz_position_unlock(this->iface);

  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void Position3dInterface::Update()
{
  player_position3d_data_t data;
  struct timeval ts;

  gz_position_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    data.pos.px = this->iface->data->pos[0];
    data.pos.py = this->iface->data->pos[1];
    data.pos.pz = this->iface->data->pos[2];

    data.pos.proll = this->iface->data->rot[0];
    data.pos.ppitch = this->iface->data->rot[1];
    data.pos.pyaw = this->iface->data->rot[2];

    data.vel.px = this->iface->data->vel_pos[0];
    data.vel.py = this->iface->data->vel_pos[1];
    data.vel.pz = this->iface->data->vel_pos[2];

    data.vel.proll = this->iface->data->vel_rot[0];
    data.vel.ppitch = this->iface->data->vel_rot[1];
    data.vel.pyaw = this->iface->data->vel_rot[2];

    data.stall = (uint8_t) this->iface->data->stall;

    this->driver->Publish( this->device_addr, NULL,
                   PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE, 
                   (void*)&data, sizeof(data), &this->datatime );
 
  }

  gz_position_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void Position3dInterface::Subscribe()
{
  // Open the interface
  if (gz_position_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void Position3dInterface::Unsubscribe()
{
  gz_position_close(this->iface);
}
