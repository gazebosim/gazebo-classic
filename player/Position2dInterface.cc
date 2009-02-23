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
/* Desc: Position Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Position2d Interface
- PLAYER_POSITION2D_CMD_VEL
- PLAYER_POSITION2D_REQ_SET_ODOM
- PLAYER_POSITION2D_REQ_MOTOR_POWER
*/

/* TODO
- PLAYER_POSITION2D_REQ_GET_GEOM
- PLAYER_POSITION2D_REQ_RESET_ODOM
*/
#include <math.h>

#include "GazeboError.hh"
#include "gazebo.h"
#include "GazeboDriver.hh"
#include "Position2dInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
Position2dInterface::Position2dInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new PositionIface();

  this->datatime = -1;

}

///////////////////////////////////////////////////////////////////////////////
// Destructor
Position2dInterface::~Position2dInterface()
{
  // Release this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int Position2dInterface::ProcessMessage(QueuePointer &respQueue,
                                        player_msghdr_t *hdr, void *data)
{
  if (this->iface->Lock(1))
  {

    // COMMAND VELOCITY:
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                              PLAYER_POSITION2D_CMD_VEL, this->device_addr))
    {
      player_position2d_cmd_vel_t *cmd;

      cmd = (player_position2d_cmd_vel_t*) data;

      this->iface->data->cmdVelocity.pos.x = cmd->vel.px;
      this->iface->data->cmdVelocity.pos.y = cmd->vel.py;
      this->iface->data->cmdVelocity.yaw = cmd->vel.pa;

      return 0;
    }

    // REQUEST SET ODOMETRY
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_SET_ODOM, this->device_addr))
    {
      if (hdr->size != sizeof(player_position2d_set_odom_req_t))
      {
        PLAYER_WARN("Arg to odometry set requestes wrong size; ignoring");
        return -1;
      }

      player_position2d_set_odom_req_t *odom = (player_position2d_set_odom_req_t*)data;

      this->iface->data->pose.pos.x = odom->pose.px;
      this->iface->data->pose.pos.y = odom->pose.py;
      this->iface->data->pose.yaw = odom->pose.pa;

      this->driver->Publish(this->device_addr, respQueue,
                            PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);

      return 0;
    }

    // CMD Set Motor Power
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                   PLAYER_POSITION2D_REQ_MOTOR_POWER, this->device_addr))
    {
      // TODO
      return 0;
    }

    // REQUEST SET MOTOR POWER
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_MOTOR_POWER, this->device_addr))
    {
      if (hdr->size != sizeof(player_position2d_power_config_t))
      {
        PLAYER_WARN("Arg to motor set requestes wrong size; ignoring");
        return -1;
      }

      player_position2d_power_config_t *power;

      power = (player_position2d_power_config_t*) data;

      this->iface->data->cmdEnableMotors = power->state;

      this->driver->Publish(this->device_addr, respQueue,
                            PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);

      return 0;
    }

    // REQUEST GET GEOMETRY
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_GET_GEOM, this->device_addr))
    {
      if (hdr->size != 0)
      {
        PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
        return -1;
      }

      player_position2d_geom_t geom;

      // TODO: get correct dimensions; there are for the P2AT

      geom.pose.px = 0;
      geom.pose.py = 0;
      geom.pose.pz = 0;
      geom.pose.pyaw = 0;
      geom.pose.ppitch = 0;
      geom.pose.proll = 0;
      geom.size.sw= 0.53;
      geom.size.sl = 0.38;
      geom.size.sh = 0.31;

      this->driver->Publish(this->device_addr, respQueue,
                            PLAYER_MSGTYPE_RESP_ACK,
                            PLAYER_POSITION2D_REQ_GET_GEOM,
                            &geom, sizeof(geom), NULL);

      return 0;
    }

    // REQUEST RESET ODOMETRY
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                   PLAYER_POSITION2D_REQ_RESET_ODOM, this->device_addr))
    {
      if (hdr->size != 0)
      {
        PLAYER_WARN("Arg reset position request is wrong size; ignoring");
        return -1;
      }

      // TODO: Make this work!!
      //
      this->driver->Publish(this->device_addr, respQueue,
                            PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_RESET_ODOM);

      return 0;
    }
    else
    {
      printf("Unhandled\n");
    }

    this->iface->Unlock();
  }
  else
  {
    printf("unable to get lock\n");
    this->Unsubscribe();
  }

  return -1;

}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void Position2dInterface::Update()
{
  player_position2d_data_t data;
  struct timeval ts;

  memset(&data, 0, sizeof(data));

  if (this->iface->Lock(1))
  {
    // Only Update when new data is present
    if (this->iface->data->head.time > this->datatime)
    {
      this->datatime = this->iface->data->head.time;

      ts.tv_sec = (int) (this->iface->data->head.time);
      ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

      data.pos.px = this->iface->data->pose.pos.x;
      data.pos.py = this->iface->data->pose.pos.y;
      data.pos.pa = this->iface->data->pose.yaw;

      data.vel.px = this->iface->data->velocity.pos.x;
      data.vel.py = this->iface->data->velocity.pos.y;
      data.vel.pa = this->iface->data->velocity.yaw;

      data.stall = (uint8_t) this->iface->data->stall;

      this->driver->Publish( this->device_addr,
                             PLAYER_MSGTYPE_DATA,
                             PLAYER_POSITION2D_DATA_STATE,
                             (void*)&data, sizeof(data), &this->datatime );

    }

    this->iface->Unlock();
  }
  else
    this->Unsubscribe();
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void Position2dInterface::Subscribe()
{
  // Open the interface
  try
  {
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout <<"Error Subscribing to Gazebo Position2d Interface\n"
    << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void Position2dInterface::Unsubscribe()
{
  this->iface->Close();
}
