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
/* Desc: Bumper Interface for Player
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * CVS: $Id:$
 */

/**
@addtogroup player
@par Bumper Interface
- PLAYER_BUMPER_DATA_STATE
*/

/* TODO
- PLAYER_BUMPER_REQ_GET_GEOM
*/

#include <iostream>
#include <math.h>

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "BumperInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
BumperInterface::BumperInterface(player_devaddr_t addr,
                                 GazeboDriver *driver, ConfigFile *cf, 
                                 int section)
    : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new BumperIface();

  this->datatime = -1;

  this->data.bumpers_count = 1;
  this->data.bumpers = new uint8_t[1];
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
BumperInterface::~BumperInterface()
{
  // Release this interface
  delete this->iface;

  delete [] this->data.bumpers;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int BumperInterface::ProcessMessage(QueuePointer &respQueue,
                                     player_msghdr_t *hdr, void *data)
{
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void BumperInterface::Update()
{
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    this->datatime = this->iface->data->head.time;

    if (this->data.bumpers_count < this->iface->data->bumper_count)
    {
      delete [] this->data.bumpers;
      this->data.bumpers = new uint8_t[this->iface->data->bumper_count];
    }

    // Copy bumper data
    this->data.bumpers_count = this->iface->data->bumper_count;

    memcpy( this->data.bumpers, this->iface->data->bumpers, 
            sizeof(uint8_t) * this->data.bumpers_count );

    this->driver->Publish( this->device_addr,
                           PLAYER_MSGTYPE_DATA,
                           PLAYER_BUMPER_DATA_STATE,
                           (void*)&data, sizeof(data), &this->datatime );
  }

  this->iface->Unlock();
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void BumperInterface::Subscribe()
{
  try
  {
    this->iface->Open( GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    std::cerr << "Error subscribing to Gazebo Bumper Interface\n"
      << e << "\n";
    exit(0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void BumperInterface::Unsubscribe()
{
  this->iface->Close();
}
