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
/* Desc: Bumper Interface for Player
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 */

/**
  @addtogroup player
  @par Bumper Interface
  - PLAYER_BUMPER_DATA_STATE
  */

/* TODO
   - PLAYER_BUMPER_REQ_GET_GEOM
   */

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "BumperInterface.hh"

using namespace libgazebo;
boost::recursive_mutex *BumperInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
BumperInterface::BumperInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf,
    int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new BumperIface();

  this->datatime = -1;

  this->data.bumpers_count = 1;
  this->data.bumpers = new uint8_t[1];

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
BumperInterface::~BumperInterface()
{
  /*
  // Release this interface
  delete this->iface;

  delete [] this->data.bumpers;
  */
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
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
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

    memcpy(this->data.bumpers, this->iface->data->bumpers,
        sizeof(uint8_t) * this->data.bumpers_count);

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_BUMPER_DATA_STATE,
        (void*)&data, sizeof(data), &this->datatime);
  }

  this->iface->Unlock();
  */
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received. This is called from
// GazeboDriver::Subscribe
void BumperInterface::Subscribe()
{
  /*
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string &e)
  {
    std::cerr << "Error subscribing to Gazebo Bumper Interface\n"
      << e << "\n";
    exit(0);
  }
  */
}

///////////////////////////////////////////////////////////////////////////////
// Close a SHM interface. This is called from GazeboDriver::Unsubscribe
void BumperInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
