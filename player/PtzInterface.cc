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
/* Desc: Ptz Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Ptz Interface
- PLAYER_PTZ_CMD_STATE
*/

/* TODO
PLAYER_PTZ_REQ_GEOM
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "PtzInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
PtzInterface::PtzInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_ptz_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
PtzInterface::~PtzInterface()
{
  // Release this interface
  gz_ptz_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int PtzInterface::ProcessMessage(MessageQueue *respQueue,
    player_msghdr_t *hdr, void *data)
{

  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
        PLAYER_PTZ_CMD_STATE, this->device_addr))
  {
    player_ptz_cmd_t *cmd;

    assert(hdr->size >= sizeof(player_ptz_cmd_t));

    cmd = (player_ptz_cmd_t*) data;

    gz_ptz_lock(this->iface, 1);

    this->iface->data->cmd_pan = cmd->pan;
    this->iface->data->cmd_tilt = cmd->tilt;
    this->iface->data->cmd_zoom = cmd->zoom;

    gz_ptz_unlock(this->iface);
  }

  // Is it a request for ptz geometry?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_PTZ_REQ_GEOM, this->device_addr))
  {
    if(hdr->size == 0)
    {
      // TODO: Implement this
      player_ptz_geom_t pgeom;

      pgeom.pos.px = 0;
      pgeom.pos.py = 0;
      pgeom.pos.pz = 0;
      pgeom.pos.proll = 0;
      pgeom.pos.ppitch = 0;
      pgeom.pos.pyaw   =0;

      pgeom.size.sl = 0; 
      pgeom.size.sw = 0; 
      pgeom.size.sh = 0; // same as sl.  

      this->driver->Publish( this->device_addr, respQueue,
         PLAYER_MSGTYPE_RESP_ACK, 
         PLAYER_PTZ_REQ_GEOM,
         (void*)&pgeom, sizeof(pgeom), NULL );

      return(0);
    }
  }

  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void PtzInterface::Update()
{
  player_ptz_data_t data;
  struct timeval ts;

  gz_ptz_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    int i;
    float rangeRes;
    float angleRes;

    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    data.pan = this->iface->data->pan;
    data.tilt = this->iface->data->tilt;
    data.zoom = this->iface->data->zoom;

    this->driver->Publish( this->device_addr, NULL,
                   PLAYER_MSGTYPE_DATA,
                   PLAYER_PTZ_DATA_STATE,      
                   (void*)&data, sizeof(data), &this->datatime );
 

  }

  gz_ptz_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void PtzInterface::Subscribe()
{
  // Open the interface
  if (gz_ptz_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void PtzInterface::Unsubscribe()
{
  gz_ptz_close(this->iface);
}
