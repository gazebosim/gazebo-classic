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
/* Desc: Opaque Interface for Player
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 */

/**
@addtogroup player
@par Opaque Interface
*/
/* TODO
Do we need these?
- PLAYER_OPAQUE_DATA_STATE
- PLAYER_OPAQUE_CMD_DATA
- PLAYER_OPAQUE_REQ_DATA
*/
#include <math.h>

#include "GazeboError.hh"
#include "gazebo.h"
#include "GazeboDriver.hh"
#include "OpaqueInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
OpaqueInterface::OpaqueInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new OpaqueIface();

  this->datatime = -1;
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
OpaqueInterface::~OpaqueInterface()
{
  // Release this interface
  delete this->iface;
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int OpaqueInterface::ProcessMessage(QueuePointer &respQueue,
                   player_msghdr_t *hdr, void *data)
{
  if (this->iface->Lock(1))
  {
    // nothing yet
      return 0;
  }
  else
    this->Unsubscribe();

  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void OpaqueInterface::Update()
{
  player_opaque_data_t data;
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

        data.data_count = this->iface->data->data_count;
        data.data = this->iface->data->data;

        this->driver->Publish( this->device_addr,
          PLAYER_MSGTYPE_DATA,
          PLAYER_OPAQUE_DATA_STATE,
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
void OpaqueInterface::Subscribe()
{
  // Open the interface
  try
  {
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string e)
  {
    //std::ostringstream stream;
    std::cout <<"Error Subscribing to Gazebo Opaque Interface\n"
           << e << "\n";
    //gzthrow(stream.str());
    exit(0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void OpaqueInterface::Unsubscribe()
{
  this->iface->Close();
}
