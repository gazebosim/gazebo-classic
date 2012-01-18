/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Laser Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/* TODO
*/

#include <math.h>
#include <iostream>

#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "LaserInterface.hh"

using namespace libgazebo;
boost::recursive_mutex *LaserInterface::mutex = NULL;

/////////////////////////////////////////////////
LaserInterface::LaserInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new LaserIface();

  this->scanId = 0;

  this->datatime = -1;

  memset(&this->data, 0, sizeof(this->data));

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
*/
}

/////////////////////////////////////////////////
LaserInterface::~LaserInterface()
{
  /*
  player_laser_data_t_cleanup(&this->data);

  // Release this interface
  delete this->iface;
  */
}

/////////////////////////////////////////////////
int LaserInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  // Is it a request to set the laser's config?
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_LASER_REQ_SET_CONFIG,
        this->device_addr))
  {
    // player_laser_config_t* plc = (player_laser_config_t*)data;

    if (hdr->size == sizeof(player_laser_config_t))
    {
      // TODO: Complete this

      this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK,
          PLAYER_LASER_REQ_SET_CONFIG);
      return(0);
    }
    else
    {
      printf("config request len is invalid (%d != %d)",
          (int)hdr->size, (int)sizeof(player_laser_config_t));

      return(-1);
    }
  }

  // Is it a request to get the laser's config?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_LASER_REQ_GET_CONFIG,
        this->device_addr))
  {
    if (hdr->size == 0)
    {
      int intensity = 1; // todo

      player_laser_config_t plc;
      memset(&plc, 0, sizeof(plc));

      plc.min_angle = this->iface->data->min_angle;
      plc.max_angle = this->iface->data->max_angle;
      plc.max_range = this->iface->data->max_range;
      plc.resolution = this->iface->data->res_angle;
      plc.range_res = this->iface->data->res_range;
      plc.intensity = intensity;

      this->driver->Publish(this->device_addr, respQueue,
          PLAYER_MSGTYPE_RESP_ACK,
          PLAYER_LASER_REQ_GET_CONFIG,
          (void*)&plc, sizeof(plc), NULL);
      return(0);
    }
    else
    {
      printf("config request len is invalid (%d != %d)", (int)hdr->size, 0);
      return(-1);
    }
  }


  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_LASER_REQ_GET_GEOM, this->device_addr))
  {
    player_laser_geom_t rep;


    // TODO: get geometry from somewhere
    memset(&rep.pose, 0, sizeof(rep.pose));
    memset(&rep.size, 0, sizeof(rep.size));

    rep.pose.px = this->iface->data->pose.pos.x;
    rep.pose.py = this->iface->data->pose.pos.y;
    rep.pose.pyaw = this->iface->data->pose.yaw;

    rep.size.sw = this->iface->data->size.x;
    rep.size.sl = this->iface->data->size.y;

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK,
        PLAYER_LASER_REQ_GET_GEOM,
        &rep, sizeof(rep), NULL);

    return 0;
  }
*/
  return -1;
}

/////////////////////////////////////////////////
void LaserInterface::Update()
{
  /*
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->iface->Lock(1))
  {
    // Only Update when new data is present
    if (this->iface->data->head.time > this->datatime)
    {
      int i;
      float rangeRes;
      float angleRes;

      this->datatime = this->iface->data->head.time;

      ts.tv_sec = (int) (this->iface->data->head.time);
      ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);

      rangeRes = this->iface->data->res_range;
      angleRes = this->iface->data->res_angle;

      double oldCount = this->data.ranges_count;

      this->data.min_angle = this->iface->data->min_angle;
      this->data.max_angle = this->iface->data->max_angle;
      this->data.resolution = angleRes;
      this->data.max_range = this->iface->data->max_range;
      this->data.ranges_count =
        this->data.intensity_count = this->iface->data->range_count;
      this->data.id = this->scanId++;

      if (oldCount != this->data.ranges_count)
      {
        delete [] this->data.ranges;
        delete [] this->data.intensity;

        this->data.ranges = new float[data.ranges_count];
        this->data.intensity = new uint8_t[data.intensity_count];
      }

      for (i = 0; i < this->iface->data->range_count; i++)
      {
        this->data.ranges[i] = this->iface->data->ranges[i] / rangeRes;
        this->data.intensity[i] =
          (uint8_t) (int) this->iface->data->intensity[i];
      }

      if (this->data.ranges_count > 0)
      {
        this->driver->Publish(this->device_addr,
            PLAYER_MSGTYPE_DATA,
            PLAYER_LASER_DATA_SCAN,
            (void*)&this->data, sizeof(this->data), &this->datatime);
      }
    }

    this->iface->Unlock();
  }
  else
    this->Unsubscribe();
    */
}

/////////////////////////////////////////////////
void LaserInterface::Subscribe()
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
    std::cout << "Error Subscribing to Gazebo Laser Interface\n"
      << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

/////////////////////////////////////////////////
void LaserInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
