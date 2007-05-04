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
/* Desc: Gps Interface for Player
 * Author: Glenn Laguna
 * Date: 20 June 2006
 * CVS: $Id$
 */

/**
@addtogroup player
@par Gps Interface
- PLAYER_GPS_DATA_STATE
*/


#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "GpsInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
GpsInterface::GpsInterface(player_devaddr_t addr, GazeboDriver *driver, 
                          ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a GPS Interface
  this->iface = gz_gps_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
GpsInterface::~GpsInterface()
{
  // Release this interface
  gz_gps_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages.  This is called from GazeboDriver
int GpsInterface::ProcessMessage(MessageQueue *respQueue, player_msghdr_t *hdr,
                             void *data)
{
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info.  This is
// called from GazeboDriver::Update
void GpsInterface::Update()
{
  player_gps_data_t data;
  double e[3];
  struct timeval ts;

  gz_gps_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    data.latitude =  (int32_t) (1e7 * this->iface->data->latitude);
    data.longitude = (int32_t) (1e7 * this->iface->data->longitude);
    data.altitude =  (int32_t) this->iface->data->altitude;
    data.utm_e = this->iface->data->utm_e;
    data.utm_n = this->iface->data->utm_n;       
    // data.quality = this->iface->data->quality;     
    // data.hdop = (uint32_t) this->iface->data->hdop; 
    // data.vdop = (uint32_t) this->iface->data->vdop;
    // data.err_horz = this->iface->data->err_horz;
    // data.err_vert = this->iface->data->err_vert;

    this->driver->Publish( this->device_addr, NULL, PLAYER_MSGTYPE_DATA, 
        PLAYER_GPS_DATA_STATE, (void*)&data, 
        sizeof(data), &this->datatime);

  }

  gz_gps_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received.  This is called from
// GazeboDriver::Subscribe
void GpsInterface::Subscribe()
{
  // Open the interface
  if (gz_gps_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a GPS interface.  This is called from GazeboDriver::Unsubscribe
void GpsInterface::Unsubscribe()
{
  gz_gps_close(this->iface);
}
