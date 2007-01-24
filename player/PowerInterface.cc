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
 * CVS: $Id: PowerInterface.cc,v 1.2.2.1 2006/12/16 22:43:22 natepak Exp $
 */

/**
@addtogroup player
@par Power Interface
*/

#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "PowerInterface.hh"

///////////////////////////////////////////////////////////////////////////////
// Constructor
PowerInterface::PowerInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = gz_power_alloc();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
PowerInterface::~PowerInterface()
{
  // Release this interface
  gz_power_free(this->iface); 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int PowerInterface::ProcessMessage(MessageQueue *respQueue,
                   player_msghdr_t *hdr, void *data)
{
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void PowerInterface::Update()
{
  player_power_data_t data;
  struct timeval ts;

  gz_power_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->time > this->datatime)
  {
    this->datatime = this->iface->data->time;

    ts.tv_sec = (int) (this->iface->data->time);
    ts.tv_usec = (int) (fmod(this->iface->data->time, 1) * 1e6);

    data.percent = this->iface->data->levels[0];
    
    this->driver->Publish( this->device_addr, NULL,
                           PLAYER_MSGTYPE_DATA,
                           PLAYER_POWER_DATA_STATE,
                           (void*)&data, sizeof(data), &this->datatime );
  }

  gz_power_unlock(this->iface);
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void PowerInterface::Subscribe()
{
  // Open the interface
  if (gz_power_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Power Interface\n");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void PowerInterface::Unsubscribe()
{
  gz_power_close(this->iface);
}
