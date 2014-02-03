/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "OpaqueInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *OpaqueInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
OpaqueInterface::OpaqueInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new OpaqueIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
OpaqueInterface::~OpaqueInterface()
{
  /*
  // Release this interface
  delete this->iface;
  */
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages. This is called from GazeboDriver
int OpaqueInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->iface->Lock(1))
  {
    // nothing yet
    return 0;
  }
  else
    this->Unsubscribe();
*/
  return -1;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info. This is
// called from GazeboDriver::Update
void OpaqueInterface::Update()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
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

      this->driver->Publish(this->device_addr,
          PLAYER_MSGTYPE_DATA,
          PLAYER_OPAQUE_DATA_STATE,
          (void*)&data, sizeof(data), &this->datatime);
    }

    this->iface->Unlock();
  }
  else
    this->Unsubscribe();
    */
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void OpaqueInterface::Subscribe()
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
    std::cout <<"Error Subscribing to Gazebo Opaque Interface\n"
      << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void OpaqueInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
