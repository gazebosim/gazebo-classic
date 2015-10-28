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
/* Desc: Sonar Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/**
  @addtogroup player
  @par Sonar Interface
  - PLAYER_SONAR_REQ_GET_GEOM
  */

/* TODO
   PLAYER_SONAR_REQ_POWER
   */

#include <math.h>

#include "GazeboDriver.hh"
#include "SonarInterface.hh"

/////////////////////////////////////////////////
SonarInterface::SonarInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
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

  // Allocate a Position Interface
  this->iface = gz_sonar_alloc();

  this->datatime = -1;
  */
}

/////////////////////////////////////////////////
SonarInterface::~SonarInterface()
{
  /*
  // Release this interface
  gz_sonar_free(this->iface);
  */
}

/////////////////////////////////////////////////
int SonarInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_SONAR_REQ_GET_GEOM, this->device_addr))
  {
    player_sonar_geom_t geom;

    gz_sonar_lock(this->iface, 1);

    geom.poses_count = this->iface->data->sonar_count;

    // the position of valid sonar
    for (int i = 0; i < this->iface->data->sonar_count; i++)
    {
      geom.poses[i].px = this->iface->data->sonar_pos[i][0];
      geom.poses[i].py = this->iface->data->sonar_pos[i][1];
      geom.poses[i].pa = this->iface->data->sonar_rot[i][2];
    }
    gz_sonar_unlock(this->iface);

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK,
        PLAYER_SONAR_REQ_GET_GEOM,
        &geom, sizeof(geom), NULL);

    return 0;
  }

  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_SONAR_REQ_POWER, this->device_addr))
  {
    player_sonar_power_config_t *power;

    assert((size_t) hdr->size >= sizeof(player_sonar_power_config_t));
    power = (player_sonar_power_config_t*) data;

    gz_sonar_lock(this->iface, 1);

    // TODO

    gz_sonar_unlock(this->iface);

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK,
        PLAYER_SONAR_REQ_POWER,
        power, sizeof(*power), NULL);

    return 0;
  }
  */

  return -1;
}

/////////////////////////////////////////////////
void SonarInterface::Update()
{
  /*
  player_sonar_data_t data;
  struct timeval ts;

  gz_sonar_lock(this->iface, 1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {

    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

    memset(&data, 0, sizeof(data));

    data.ranges_count = this->iface->data->sonar_count;

    for (int i = 0; i < this->iface->data->sonar_count; i++)
      data.ranges[i] = this->iface->data->sonar_ranges[i];

    this->driver->Publish(this->device_addr, NULL,
        PLAYER_MSGTYPE_DATA,
        PLAYER_SONAR_DATA_RANGES,
        (void*)&data, sizeof(data), &this->datatime);
  }

  gz_sonar_unlock(this->iface);
  */
}

/////////////////////////////////////////////////
void SonarInterface::Subscribe()
{
  /*
  // Open the interface
  if (gz_sonar_open(this->iface, GazeboClient::client, this->gz_id) != 0)
  {
    printf("Error Subscribing to Gazebo Position Interface\n");
  }
  */
}

/////////////////////////////////////////////////
void SonarInterface::Unsubscribe()
{
  /*
  gz_sonar_close(this->iface);
  */
}
