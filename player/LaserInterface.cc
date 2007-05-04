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
/* Desc: Laser Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Laser Interface
- PLAYER_LASER_REQ_SET_CONFIG
- PLAYER_LASER_REQ_GET_CONFIG
*/

/* TODO
PLAYER_LASER_REQ_GET_GEOM
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "LaserInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
LaserInterface::LaserInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_laser_alloc();

  this->scanId = 0;

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
LaserInterface::~LaserInterface()
{
  // Release this interface
  gz_laser_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int LaserInterface::ProcessMessage(MessageQueue *respQueue,
                   player_msghdr_t *hdr, void *data)
{
  // Is it a request to set the laser's config?
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
                           PLAYER_LASER_REQ_SET_CONFIG, 
                           this->device_addr))
  {
    player_laser_config_t* plc = (player_laser_config_t*)data;

    if( hdr->size == sizeof(player_laser_config_t) )
    {
      // TODO: Complete this

        this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK, 
          PLAYER_LASER_REQ_SET_CONFIG);
      return(0);
    }
    else
    {
      printf("config request len is invalid (%d != %d)", 
          (int)hdr->size, (int)sizeof(player_laser_config_t));

      return(-1);
    }
  }

  // Is it a request to get the laser's config?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_LASER_REQ_GET_CONFIG,
                                 this->device_addr))
  {   
    if( hdr->size == 0 )
    {
      int intensity = 1; // todo

      player_laser_config_t plc;
      memset(&plc,0,sizeof(plc));

      plc.min_angle = this->iface->data->min_angle;
      plc.max_angle = this->iface->data->max_angle;
      plc.max_range = this->iface->data->max_range;
      plc.resolution = this->iface->data->res_angle;
      plc.intensity = intensity;

      this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK, 
			    PLAYER_LASER_REQ_GET_CONFIG,
			    (void*)&plc, sizeof(plc), NULL);
      return(0);
    }
    else
    {
      printf("config request len is invalid (%d != %d)", (int)hdr->size,0);
      return(-1);
    }
  }


  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_LASER_REQ_GET_GEOM, this->device_addr))
  {
    player_laser_geom_t rep;

    // TODO: get geometry from somewhere
    rep.pose.px = 0.0;
    rep.pose.py = 0.0;
    rep.pose.pa = 0.0;
    rep.size.sw = 0.0;
    rep.size.sl = 0.0;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, 
        PLAYER_LASER_REQ_GET_GEOM, 
        &rep, sizeof(rep), NULL);

    return 0;
  }


  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void LaserInterface::Update()
{
  player_laser_data_t data;
  struct timeval ts;

  gz_laser_lock(this->iface, 1);

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

    // Pick the rage resolution to use (1, 10, 100)
    if (this->iface->data->max_range <= 8.192)
      rangeRes = 1.0;
    else if (this->iface->data->max_range <= 81.92)
      rangeRes = 10.0;
    else
      rangeRes = 100.0;

    angleRes = this->iface->data->res_angle;

    //printf("range res = %f %f\n", rangeRes, this->iface->data->max_range);
  
    data.min_angle = this->iface->data->min_angle;
    data.max_angle = this->iface->data->max_angle;
    data.max_range = this->iface->data->max_range;
    data.resolution = angleRes;
    data.ranges_count = data.ranges_count = this->iface->data->range_count; 
    data.id = this->scanId++;

    for (i = 0; i < this->iface->data->range_count; i++)
    {
      data.ranges[i] = this->iface->data->ranges[i] / rangeRes;
      data.intensity[i] = (uint8_t) (int) this->iface->data->intensity[i];
    }

    this->driver->Publish( this->device_addr, NULL,
                   PLAYER_MSGTYPE_DATA,
                   PLAYER_LASER_DATA_SCAN,        
                   (void*)&data, sizeof(data), &this->datatime );
  }

  gz_laser_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void LaserInterface::Subscribe()
{
  // Open the interface
  if (gz_laser_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void LaserInterface::Unsubscribe()
{
  gz_laser_close(this->iface);
}
