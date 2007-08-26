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
 * Date: 30 Jan 2007
 * SVN: $Id$
 */

/**
@addtogroup player
@par Graphics3d Interface
- PLAYER_GRAPHICS3D_CMD_CLEAR
- PLAYER_GRAPHICS3D_CMD_DRAW
*/

#include "gazebo.h"
#include "GazeboError.hh"
#include "GazeboDriver.hh"
#include "Graphics3dInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
Graphics3dInterface::Graphics3dInterface(player_devaddr_t addr, 
    GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Graphics3d Interface
  this->iface = new Graphics3dIface();
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
Graphics3dInterface::~Graphics3dInterface()
{
  // Release this interface
  delete this->iface; 
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int Graphics3dInterface::ProcessMessage(QueuePointer &/*respQueue*/,
                   player_msghdr_t *hdr, void *data)
{
  int result = -1;

  this->iface->Lock(1);

  // COMMAND CLEAR:
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
        PLAYER_GRAPHICS3D_CMD_CLEAR, this->device_addr))
  {
    player_graphics3d_cmd_draw_t *cmd;

    cmd = (player_graphics3d_cmd_draw_t*) data;

    this->iface->data->point_count = 0;

    result = 0;
  }

  // COMMAND DRAW:
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, 
        PLAYER_GRAPHICS3D_CMD_DRAW, this->device_addr))
  {
    player_graphics3d_cmd_draw_t *cmd;

    cmd = (player_graphics3d_cmd_draw_t*) data;

    this->iface->data->point_count = cmd->points_count;

    this->iface->data->color.r = cmd->color.red/255.0;
    this->iface->data->color.g = cmd->color.green/255.0;
    this->iface->data->color.b = cmd->color.blue/255.0;
    this->iface->data->color.a = cmd->color.alpha/255.0;

    for (unsigned int i=0; i<this->iface->data->point_count; i++)
    {
      this->iface->data->points[i].x = cmd->points[i].px;
      this->iface->data->points[i].y = cmd->points[i].py;
      this->iface->data->points[i].z = cmd->points[i].pz;
    }

    result = 0;
  }

  this->iface->Unlock();

  return result;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void Graphics3dInterface::Update()
{
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void Graphics3dInterface::Subscribe()
{
  // Open the interface
  try
  {
    this->iface->Open(GazeboClient::client, this->gz_id);
  } 
  catch (GazeboError e)
  {
    std::ostringstream stream;
    stream << "Error Subscribing to Gazebo Graphics3d Interface\n"
           << e << "\n";
    gzthrow(stream.str());
  }
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void Graphics3dInterface::Unsubscribe()
{
  this->iface->Close();
}
