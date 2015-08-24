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
/* Desc: Position 3d Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/* TODO
   PLAYER_POSITION3D_GET_GEOM
   PLAYER_POSITION3D_RESET_ODOM
   */

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "Position3dInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *Position3dInterface::mutex = NULL;

Position3dInterface::Position3dInterface(player_devaddr_t addr,
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
  this->iface = new PositionIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
*/
}

Position3dInterface::~Position3dInterface()
{
  /*
  // Release this interface
  delete this->iface;
  */
}

int Position3dInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // COMMAND VELOCITY:
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_POSITION3D_CMD_SET_VEL, this->device_addr))
  {
    player_position3d_cmd_vel_t *cmd;

    cmd = (player_position3d_cmd_vel_t*) data;

    this->iface->data->cmdVelocity.pos.x = cmd->vel.px;
    this->iface->data->cmdVelocity.pos.y = cmd->vel.py;
    this->iface->data->cmdVelocity.pos.z = cmd->vel.pz;

    this->iface->data->cmdVelocity.roll = cmd->vel.proll;
    this->iface->data->cmdVelocity.pitch = cmd->vel.ppitch;
    this->iface->data->cmdVelocity.yaw = cmd->vel.pyaw;

    return 0;
  }

  // REQUEST SET ODOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION3D_REQ_SET_ODOM, this->device_addr))
  {
    if (hdr->size != sizeof(player_position3d_set_odom_req_t))
    {
      PLAYER_WARN("Arg to odometry set requestes wrong size; ignoring");
      return -1;
    }

    player_position3d_set_odom_req_t *odom =
      (player_position3d_set_odom_req_t*)data;

    this->iface->data->pose.pos.x = odom->pos.px;
    this->iface->data->pose.pos.y = odom->pos.py;
    this->iface->data->pose.pos.z = odom->pos.pz;

    this->iface->data->pose.roll = odom->pos.proll;
    this->iface->data->pose.pitch = odom->pos.ppitch;
    this->iface->data->pose.yaw = odom->pos.pyaw;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_REQ_SET_ODOM);

    return 0;
  }

  // REQUEST SET MOTOR POWER
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_POSITION3D_REQ_MOTOR_POWER, this->device_addr))
  {
    if (hdr->size != sizeof(player_position3d_power_config_t))
    {
      PLAYER_WARN("Arg to motor set requestes wrong size; ignoring");
      return -1;
    }

    player_position3d_power_config_t *power;

    power = (player_position3d_power_config_t*) data;

    this->iface->data->cmdEnableMotors = power->state;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_REQ_MOTOR_POWER);

    return 0;
  }

  // REQUEST GET GEOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION3D_REQ_GET_GEOM, this->device_addr))
  {
    if (hdr->size != 0)
    {
      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
      return -1;
    }


    player_position3d_geom_t geom;

    // TODO: get correct dimensions; there are for the P2AT

    geom.pose.px = 0;
    geom.pose.py = 0;
    geom.pose.pz = 0;

    geom.size.sw = 0.53;
    geom.size.sl = 0.38;
    geom.size.sh = 0.2;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_REQ_GET_GEOM,
        &geom, sizeof(geom), NULL);

    return 0;
  }

  // REQUEST RESET ODOMETRY
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_POSITION3D_REQ_RESET_ODOM, this->device_addr))
  {
    if (hdr->size != 0)
    {
      PLAYER_WARN("Arg reset position request is wrong size; ignoring");
      return -1;
    }

    // TODO: Make this work!!
    //
    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION3D_REQ_RESET_ODOM);

    return 0;
  }

  this->iface->Unlock();

  */
  return -1;
}

void Position3dInterface::Update()
{
  /*
  player_position3d_data_t data;
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

    data.pos.px = this->iface->data->pose.pos.x;
    data.pos.py = this->iface->data->pose.pos.y;
    data.pos.pz = this->iface->data->pose.pos.z;

    data.pos.proll = this->iface->data->pose.roll;
    data.pos.ppitch = this->iface->data->pose.pitch;
    data.pos.pyaw = this->iface->data->pose.yaw;

    data.vel.px = this->iface->data->velocity.pos.x;
    data.vel.py = this->iface->data->velocity.pos.y;
    data.vel.pz = this->iface->data->velocity.pos.z;

    data.vel.proll = this->iface->data->velocity.roll;
    data.vel.ppitch = this->iface->data->velocity.pitch;
    data.vel.pyaw = this->iface->data->velocity.yaw;

    data.stall = (uint8_t) this->iface->data->stall;

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA, PLAYER_POSITION3D_DATA_STATE,
        (void*)&data, sizeof(data), &this->datatime);

  }

  this->iface->Unlock();
  */
}

void Position3dInterface::Subscribe()
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
    std::cout <<"Error Subscribing to Gazebo Position3d Interface\n"
      << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

void Position3dInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
