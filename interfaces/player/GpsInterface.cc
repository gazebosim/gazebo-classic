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
/* Desc: Gps Interface for Player
 * Author: Glenn Laguna
 * Date: 20 June 2006
 */

/**
  @addtogroup player
  @par Gps Interface
  - PLAYER_GPS_DATA_STATE
  */

#include <math.h>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "GpsInterface.hh"

boost::recursive_mutex *GpsInterface::mutex = NULL;

///////////////////////////////////////////////////////////////////////////////
// Constructor
GpsInterface::GpsInterface(player_devaddr_t addr, GazeboDriver *driver,
    ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  this->iface = NULL;
  this->gz_id = NULL;
  this->datatime = 0;

  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a GPS Interface
  this->iface = gz_gps_alloc();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
GpsInterface::~GpsInterface()
{
  /*
  // Release this interface
  gz_gps_free(this->iface);
  */
}

///////////////////////////////////////////////////////////////////////////////
// Handle all messages.  This is called from GazeboDriver
int GpsInterface::ProcessMessage(QueuePointer &respQueue, player_msghdr_t *hdr,
    void *data)
{
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Update this interface, publish new info.  This is
// called from GazeboDriver::Update
void GpsInterface::Update()
{
  /*
  player_gps_data_t data;
  // double e[3];
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
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

    this->driver->Publish(this->device_addr, NULL, PLAYER_MSGTYPE_DATA,
        PLAYER_GPS_DATA_STATE, (void*)&data,
        sizeof(data), &this->datatime);

  }

  gz_gps_unlock(this->iface);
  */
}


///////////////////////////////////////////////////////////////////////////////
// Open a SHM interface when a subscription is received.  This is called from
// GazeboDriver::Subscribe
void GpsInterface::Subscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  // Open the interface
  if (gz_gps_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
  */
}

///////////////////////////////////////////////////////////////////////////////
// Close a GPS interface.  This is called from GazeboDriver::Unsubscribe
void GpsInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  gz_gps_close(this->iface);
  */
}
