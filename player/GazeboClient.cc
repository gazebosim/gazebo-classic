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
/* Desc: Gazebo (simulator) client functions 
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <assert.h>
#include <stdio.h>
#include <libplayercore/playercore.h>

#include "GazeboTime.hh"
#include "GazeboClient.hh"

using namespace gazebo;

Client *GazeboClient::client = NULL;
SimulationIface *GazeboClient::sim = NULL;
const char *GazeboClient::prefixId = "";

extern PlayerTime* GlobalTime;

////////////////////////////////////////////////////////////////////////////////
// Initialize 
int GazeboClient::Init(int serverid, const char *prefixid)
{
  if (prefixid != NULL)
    GazeboClient::prefixId = prefixid;

  GazeboClient::client = new Client();

  // Use version 0.5.0
  if (GazeboClient::client->ConnectWait(serverid, GZ_CLIENT_ID_PLAYER) != 0)
    return -1;

  GazeboClient::sim = new SimulationIface();

  if (GazeboClient::sim->Open( GazeboClient::client, "default") != 0)
  {
    printf("Gazeob Error: Unable to connect to the Gazebo Sim Interface\n");
    return -1;
  }

  // steal the global clock - a bit aggressive, but a simple approach
  if (GlobalTime)
    delete GlobalTime;
  assert((GlobalTime = new GazeboTime()));

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Finalize
int GazeboClient::Fini()
{
  GazeboClient::sim->Close();

  if (GazeboClient::client->Disconnect() != 0)
    return -1;

  return 0;
}

