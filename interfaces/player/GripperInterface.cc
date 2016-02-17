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
/* Desc: Gripper Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/**
  @addtogroup player
  @par Gripper Interface
  - PLAYER_GRIPPER_CMD_STATE
  */

/* TODO
   - PLAYER_GRIPPER_REQ_GET_GEOM
   */

#include <math.h>

#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "GripperInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *GripperInterface::mutex = NULL;

/////////////////////////////////////////////////
GripperInterface::GripperInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section), iface(NULL), gz_id(NULL),
  datatime(0.0)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new GripperIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

/////////////////////////////////////////////////
GripperInterface::~GripperInterface()
{
  /*
  // Release this interface
  delete this->iface;
  */
}

/////////////////////////////////////////////////
int GripperInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->iface->Lock(1))
  {
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
          PLAYER_GRIPPER_CMD_OPEN, this->device_addr))
    {
      this->iface->data->cmd = GAZEBO_GRIPPER_CMD_OPEN;
      return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
          PLAYER_GRIPPER_CMD_CLOSE, this->device_addr))
    {
      this->iface->data->cmd = GAZEBO_GRIPPER_CMD_CLOSE;
      return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
          PLAYER_GRIPPER_CMD_STOP, this->device_addr))
    {
      this->iface->data->cmd = GAZEBO_GRIPPER_CMD_STOP;
      return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
          PLAYER_GRIPPER_CMD_STORE, this->device_addr))
    {
      this->iface->data->cmd = GAZEBO_GRIPPER_CMD_STORE;
      return 0;
    }
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
          PLAYER_GRIPPER_CMD_RETRIEVE, this->device_addr))
    {
      this->iface->data->cmd = GAZEBO_GRIPPER_CMD_RETRIEVE;
      return 0;
    }
    // is it a geometry request?
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
          PLAYER_GRIPPER_REQ_GET_GEOM,
          this->device_addr))
    {
      // TODO: implement me

      player_gripper_geom_t pgeom;

      pgeom.pose.px = 0;
      pgeom.pose.py = 0;
      pgeom.pose.pz = 0;
      pgeom.pose.proll = 0;
      pgeom.pose.ppitch = 0;
      pgeom.pose.pyaw = 0;

      pgeom.outer_size.sw = 0;
      pgeom.outer_size.sl = 0;
      pgeom.outer_size.sh = 0;

      pgeom.inner_size.sw = 0;
      pgeom.inner_size.sl = 0;
      pgeom.inner_size.sh = 0;

      pgeom.num_beams = 2;

      this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK,
          PLAYER_GRIPPER_REQ_GET_GEOM,
          (void*)&pgeom, sizeof(pgeom), NULL);

      return 0;
    }

    this->iface->Unlock();
  }
  else
    this->Unsubscribe();

    */
  return -1;
}

/////////////////////////////////////////////////
void GripperInterface::Update()
{
  /*
  player_gripper_data_t data;
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

    // break beams are now implemented
    data.beams = 0;

    data.beams |= this->iface->data->grip_limit_reach ? 0x01 : 0x00;
    data.beams |= this->iface->data->lift_limit_reach ? 0x02 : 0x00;
    data.beams |= this->iface->data->outer_beam_obstruct ? 0x04 : 0x00;
    data.beams |= this->iface->data->inner_beam_obstruct ? 0x08 : 0x00;
    data.beams |= this->iface->data->left_paddle_open ? 0x10 : 0x00;
    data.beams |= this->iface->data->right_paddle_open ? 0x20 : 0x00;

    // This works with player cvs.
#ifdef PLAYER_GRIPPER_STATE_OPEN
    // set the proper state
    if (this->iface->data->state == GAZEBO_GRIPPER_STATE_OPEN)
      data.state = PLAYER_GRIPPER_STATE_OPEN;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_CLOSED)
      data.state = PLAYER_GRIPPER_STATE_CLOSED;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_MOVING)
      data.state = PLAYER_GRIPPER_STATE_MOVING;
    else if (this->iface->data->state == GAZEBO_GRIPPER_STATE_ERROR)
      data.state = PLAYER_GRIPPER_STATE_ERROR;
#endif

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_GRIPPER_DATA_STATE,
        (void*)&data, sizeof(data), &this->datatime);
  }

  this->iface->Unlock();
  */
}

/////////////////////////////////////////////////
void GripperInterface::Subscribe()
{
  /*
  try
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);
    this->iface->Open(GazeboClient::client, this->gz_id);
  }
  catch (std::string &e)
  {
    std::cerr << "Error subscribing to Gazebo Gripper Interface\n"
      << e << "\n";
    exit(0);
  }
  */
}

/////////////////////////////////////////////////
void GripperInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
