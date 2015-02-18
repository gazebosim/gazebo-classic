/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: PTZ Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/**
  @addtogroup player
  @par PTZ Interface
  - PLAYER_PTZ_CMD_STATE
  */

/* TODO
   PLAYER_PTZ_REQ_GEOM
   */

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "PTZInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *PTZInterface::mutex = NULL;

/////////////////////////////////////////////////
PTZInterface::PTZInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new PTZIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

/////////////////////////////////////////////////
PTZInterface::~PTZInterface()
{
  /*
  // Release this interface
  delete this->iface;
  */
}

/////////////////////////////////////////////////
int PTZInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_PTZ_CMD_STATE, this->device_addr))
  {
    player_ptz_cmd_t *cmd;

    assert(hdr->size >= sizeof(player_ptz_cmd_t));

    cmd = (player_ptz_cmd_t*) data;

    this->iface->Lock(1);

    this->iface->data->cmd_pan = cmd->pan;
    this->iface->data->cmd_tilt = cmd->tilt;
    this->iface->data->cmd_zoom = cmd->zoom;
    this->iface->data->cmd_tilt_speed = cmd->tiltspeed;
    this->iface->data->cmd_pan_speed = cmd->panspeed;

    this->iface->Unlock();

    return 0;
  }

  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_PTZ_REQ_CONTROL_MODE, this->device_addr))
  {
    player_ptz_req_control_mode_t *req;

    assert(hdr->size >= sizeof(player_ptz_req_control_mode_t));

    req = (player_ptz_req_control_mode_t *) data;

    if (req->mode == PLAYER_PTZ_VELOCITY_CONTROL)
      this->iface->data->control_mode = GAZEBO_PTZ_VELOCITY_CONTROL;
    else
      this->iface->data->control_mode = GAZEBO_PTZ_POSITION_CONTROL;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK,
        PLAYER_PTZ_REQ_CONTROL_MODE);
    return(0);
  }

  // Is it a request for ptz geometry?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_PTZ_REQ_GEOM, this->device_addr))
  {
    if (hdr->size == 0)
    {
      // TODO: Implement this
      player_ptz_geom_t pgeom;

      pgeom.pos.px = 0;
      pgeom.pos.py = 0;
      pgeom.pos.pz = 0;
      pgeom.pos.proll = 0;
      pgeom.pos.ppitch = 0;
      pgeom.pos.pyaw   = 0;

      pgeom.size.sl = 0;
      pgeom.size.sw = 0;
      pgeom.size.sh = 0; // same as sl.

      this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK,
          PLAYER_PTZ_REQ_GEOM,
          (void*)&pgeom, sizeof(pgeom), NULL);

      return(0);
    }
  }

*/
  return -1;
}

/////////////////////////////////////////////////
void PTZInterface::Update()
{
  /*
  player_ptz_data_t data;
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    data.pan = this->iface->data->pan;
    data.tilt = this->iface->data->tilt;
    data.zoom = this->iface->data->zoom;

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_PTZ_DATA_STATE,
        (void*)&data, sizeof(data), &this->datatime);
  }

  this->iface->Unlock();
  */
}

/////////////////////////////////////////////////
void PTZInterface::Subscribe()
{
  /*
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string &e)
  {
    // std::ostringstream stream;
    std::cout << "Error subscribing to Gazebo PTZ Interface\n" << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

/////////////////////////////////////////////////
void PTZInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
