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
/* Desc: Simulation Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include "gazebo.h"
#include "GazeboDriver.hh"
#include "SimulationInterface.hh"

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
// Constructor
SimulationInterface::SimulationInterface(player_devaddr_t addr, GazeboDriver *driver, ConfigFile *cf, int section)
  : GazeboInterface(addr, driver, cf, section)
{
  // ID of the server
  int serverId = atoi((char*)cf->ReadString(section,"server_id","default"));

  // Initialize the Client. Creates the SHM connection
  GazeboClient::Init(serverId, "");
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
SimulationInterface::~SimulationInterface()
{
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int SimulationInterface::ProcessMessage(MessageQueue *respQueue,
                   player_msghdr_t *hdr, void *data)
{
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
        PLAYER_SIMULATION_REQ_SET_POSE2D, this->device_addr))
  {
    printf("Simulation setting pose2d\n");
  }

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void SimulationInterface::Update()
{
  return;
}

///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void SimulationInterface::Subscribe()
{
  return;
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void SimulationInterface::Unsubscribe()
{
  return;
}
