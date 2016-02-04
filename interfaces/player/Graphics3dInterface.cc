/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Desc: Position Interface for Player
 * Author: Nate Koenig
 * Date: 30 Jan 2007
 */

/**
  @addtogroup player
  @par Graphics3d Interface
  - PLAYER_GRAPHICS3D_CMD_CLEAR
  - PLAYER_GRAPHICS3D_CMD_DRAW
  */

#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "Graphics3dInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *Graphics3dInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
Graphics3dInterface::Graphics3dInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section), gz_id(NULL), iface(NULL)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Graphics3d Interface
  this->iface = new Graphics3dIface();

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
*/
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
Graphics3dInterface::~Graphics3dInterface()
{
  /*
  // Release this interface
  delete this->iface;
  */
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int Graphics3dInterface::ProcessMessage(QueuePointer &/*respQueue*/,
    player_msghdr_t *hdr, void *data)
{
  int result = -1;
  /*

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // COMMAND CLEAR:
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_GRAPHICS3D_CMD_CLEAR, this->device_addr))
  {
    // player_graphics3d_cmd_draw_t *cmd;
    // cmd = (player_graphics3d_cmd_draw_t*) data;

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

    for (unsigned int i = 0; i<this->iface->data->point_count; i++)
    {
      this->iface->data->points[i].x = cmd->points[i].px;
      this->iface->data->points[i].y = cmd->points[i].py;
      this->iface->data->points[i].z = cmd->points[i].pz;
    }

    result = 0;
  }

  this->iface->Unlock();
*/
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
  /*
  // Open the interface
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string &e)
  {
    // std::ostringstream stream;
    std::cout << "Error Subscribing to Gazebo Graphics3d Interface\n"
      << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void Graphics3dInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
