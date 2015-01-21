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
/* Desc: Actarray Interface for Player-Gazebo
 * Author: Alexis Maldonado
 * Date: 19 September 2006
 */


#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "ActarrayInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *ActarrayInterface::mutex = NULL;

// The data for the interface (gz_actarray_data_t in gazebo.h) contains
// information about each joint plus variables to store the commands. When
// a new command is sent, it modifies the cmd_pos, cmd_speed, ... variables
// and sets new_cmd to 1. The gazebo model can look at this variable to find
// out if a new command came Since joints in gazebo only support velocity
// control, the model has to implement at least a P controller to implement
// position commands. The model should support position and velocity commands.
// Home commands, brake requests, and power requests are simply translated to
// a sensible meaning in velocity or position commands.
// If the models need it, this interface should be extended.

ActarrayInterface::ActarrayInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf,
    int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = GazeboClient::prefixId + cf->ReadString(section, "gz_id", "");

  // Allocate a Actarray Interface
  this->iface = new ActarrayIface();

  memset(&this->actData, 0, sizeof(this->actData));

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

ActarrayInterface::~ActarrayInterface()
{
  /*
  delete [] this->actData.actuators;

  // Release this interface
  delete this->iface;
  */
}


int ActarrayInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_ACTARRAY_CMD_POS, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_position_cmd_t));

    player_actarray_position_cmd_t *cmd;
    cmd = (player_actarray_position_cmd_t*) data;

    this->iface->Lock(1);
    this->iface->data->cmd_pos[cmd->joint] = cmd->position;
    this->iface->data->joint_mode[cmd->joint] =
      GAZEBO_ACTARRAY_JOINT_POSITION_MODE;
    this->iface->data->new_cmd = true;
    this->iface->Unlock();

    return(0);

  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_ACTARRAY_CMD_SPEED, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_speed_cmd_t));

    player_actarray_speed_cmd_t *cmd;
    cmd = (player_actarray_speed_cmd_t*) data;

    this->iface->Lock(1);

    this->iface->data->cmd_speed[cmd->joint]= cmd->speed;
    this->iface->data->joint_mode[cmd->joint]= GAZEBO_ACTARRAY_JOINT_SPEED_MODE;

    this->iface->data->new_cmd = true;
    this->iface->Unlock();

    return(0);

  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
        PLAYER_ACTARRAY_CMD_HOME, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_home_cmd_t));
    player_actarray_home_cmd_t *cmd;
    cmd = (player_actarray_home_cmd_t*) data;

    this->iface->Lock(1);

    this->iface->data->cmd_pos[cmd->joint]= 0.0;
    this->iface->data->joint_mode[cmd->joint] =
      GAZEBO_ACTARRAY_JOINT_POSITION_MODE;
    this->iface->data->new_cmd = true;

    this->iface->Unlock();

    return(0);

  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_ACTARRAY_REQ_SPEED, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_speed_config_t));
    player_actarray_speed_config_t *req;
    player_actarray_speed_config_t response;

    req = (player_actarray_speed_config_t*)(data);

    if (req->joint < GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS)
    {
      this->iface->Lock(1);

      response.joint = req->joint;
      response.speed = this->iface->data->actuators[req->joint].speed;

      this->iface->Unlock();

      driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_REQ_SPEED, &response);

    }
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_ACTARRAY_REQ_BRAKES, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_brakes_config_t));
    player_actarray_brakes_config_t *req;
    req = (player_actarray_brakes_config_t*) data;

    // If brakes = on -> Stop all the joints. If they are off, don't do anything
    if (req->value == 1)
    {
      this->iface->Lock(1);

      for (unsigned int i = 0 ; i != GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS ; ++i)
      {
        this->iface->data->cmd_speed[i] = 0.0;
        this->iface->data->joint_mode[i] = GAZEBO_ACTARRAY_JOINT_SPEED_MODE;
        this->iface->data->new_cmd = true;

      }
      this->iface->Unlock();
    }


    driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_REQ_BRAKES);
    return(0);

  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_ACTARRAY_REQ_POWER, this->device_addr))
  {
    assert(hdr->size >= sizeof(player_actarray_power_config_t));
    player_actarray_power_config_t *req;
    req = (player_actarray_power_config_t*) data;

    // If power = off -> Stop all the joints. If power = on, don't do anything
    if (req->value == 0)
    {
      this->iface->Lock(1);

      for (unsigned int i = 0 ; i != GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS ; ++i)
      {
        this->iface->data->cmd_speed[i]= 0.0;
        this->iface->data->joint_mode[i]= GAZEBO_ACTARRAY_JOINT_SPEED_MODE;
        this->iface->data->new_cmd = true;
      }
      this->iface->Unlock();
    }

    driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_REQ_POWER);
    return(0);
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_ACTARRAY_REQ_GET_GEOM, this->device_addr))
  {
    player_actarray_geom response;
    this->iface->Lock(1);
    response.actuators_count = this->iface->data->actuators_count;

    player_actarray_actuatorgeom_t geoms[GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS];
    for (unsigned int i = 0; i < GAZEBO_ACTARRAY_MAX_NUM_ACTUATORS; ++i)
    {
      ActarrayActuatorGeom& gazeboGeom = this->iface->data->actuator_geoms[i];
      geoms[i].type = gazeboGeom.type;
      // unused
      geoms[i].length = 0;
      // unused
      memset(&(geoms[i].orientation), 0, sizeof(geoms[i].orientation));
      // unused
      memset(&(geoms[i].axis), 0, sizeof(geoms[i].axis));
      geoms[i].min = gazeboGeom.min;
      geoms[i].centre = gazeboGeom.center;
      geoms[i].max = gazeboGeom.max;
      geoms[i].home = gazeboGeom.home;
      geoms[i].config_speed = gazeboGeom.config_speed;
      geoms[i].hasbrakes = gazeboGeom.hasbrakes;
    }
    response.actuators = geoms;

    // unused
    memset(&response.base_pos, 0, sizeof(response.base_pos));
    // unused
    memset(&response.base_orientation, 0, sizeof(response.base_orientation));
    this->iface->Unlock();

    driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK, PLAYER_ACTARRAY_REQ_GET_GEOM, &response);
    return(0);
  }

  */
  return -1;
}

void ActarrayInterface::Update()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    // Update the local time so we know when new info comes
    this->datatime = this->iface->data->head.time;

    unsigned int prevCount = this->actData.actuators_count;

    // The number of actuators in the array.
    this->actData.actuators_count = this->iface->data->actuators_count;

    if (prevCount != this->actData.actuators_count)
    {
      delete [] this->actData.actuators;

      this->actData.actuators =
        new player_actarray_actuator_t[this->actData.actuators_count];
    }

    for (unsigned int i = 0; i < this->actData.actuators_count ; ++i)
    {
      float pos = this->iface->data->actuators[i].position;
      float speed = this->iface->data->actuators[i].speed;

      // float current = this->iface->data->actuators_data[i].current;
      uint8_t report_state = this->iface->data->actuators[i].state;
      this->actData.actuators[i].position = pos;
      this->actData.actuators[i].speed = speed;
      // data.actuators[i].current = current;
      this->actData.actuators[i].state = report_state;

    }

    driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_ACTARRAY_DATA_STATE,
        (void*)&this->actData, sizeof(this->actData), &this->datatime);
  }

  this->iface->Unlock();
  */
}


void ActarrayInterface::Subscribe()
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
    std::cout << "Error subscribing to Gazebo Actarray Interface\n"
              << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

void ActarrayInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
