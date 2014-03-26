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
/* Desc: Laser Interface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 */

/* TODO
   PLAYER_LASER_REQ_GET_GEOM
   */

#include <math.h>
#include <iostream>
#include <boost/thread/recursive_mutex.hpp>

#include "GazeboDriver.hh"
#include "IRInterface.hh"

using namespace libgazebo;

boost::recursive_mutex *IRInterface::mutex = NULL;

/////////////////////////////////////////////////
IRInterface::IRInterface(player_devaddr_t addr,
    GazeboDriver *driver, ConfigFile *cf, int section)
: GazeboInterface(addr, driver, cf, section)
{
  /*
  // Get the ID of the interface
  this->gz_id = (char*) calloc(1024, sizeof(char));
  strcat(this->gz_id, GazeboClient::prefixId);
  strcat(this->gz_id, cf->ReadString(section, "gz_id", ""));

  // Allocate a Position Interface
  this->iface = new IRIface();

  this->datatime = -1;

  if (this->mutex == NULL)
    this->mutex = new boost::recursive_mutex();
    */
}

/////////////////////////////////////////////////
IRInterface::~IRInterface()
{
  /*
  player_ir_data_t_cleanup(&this->data);

  // Release this interface
  delete this->iface;
  */
}

/////////////////////////////////////////////////
int IRInterface::ProcessMessage(QueuePointer &respQueue,
    player_msghdr_t *hdr, void *data)
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
        PLAYER_IR_REQ_POSE, this->device_addr))
  {
    player_ir_pose_t rep;

    // TODO: get geometry from somewhere
    rep.poses_count =  this->iface->data->ir_count;
    rep.poses = new player_pose3d_t[rep.poses_count];

    for(unsigned int i = 0;i<rep.poses_count;i++)
    {
      rep.poses[i].px = this->iface->data->poses[i].pos.x;
      rep.poses[i].py = this->iface->data->poses[i].pos.y;
      rep.poses[i].pz = this->iface->data->poses[i].pos.z;
      rep.poses[i].proll = this->iface->data->poses[i].roll;
      rep.poses[i].ppitch = this->iface->data->poses[i].pitch;
      rep.poses[i].pyaw = this->iface->data->poses[i].yaw;
    }

    this->driver->Publish(this->device_addr, respQueue,
        PLAYER_MSGTYPE_RESP_ACK,
        PLAYER_IR_REQ_POSE,
        &rep, sizeof(rep), NULL);
    delete []rep.poses;
    return 0;
  }
  */

  return -1;
}

/////////////////////////////////////////////////
void IRInterface::Update()
{
  /*
  struct timeval ts;

  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Lock(1);

  // Only Update when new data is present
  if (this->iface->data->head.time > this->datatime)
  {
    this->datatime = this->iface->data->head.time;

    ts.tv_sec = (int) (this->iface->data->head.time);
    ts.tv_usec = (int) (fmod(this->iface->data->head.time, 1) * 1e6);


    int oldCount = this->data.ranges_count;
    this->data.ranges_count = this->iface->data->ir_count;
    this->data.voltages_count = this->iface->data->ir_count;

    if(oldCount != (int)this->data.ranges_count)
    {
      delete []this->data.ranges;
      delete []this->data.voltages;

      this->data.ranges = new float[data.ranges_count];
      this->data.voltages = new float[data.ranges_count];
    }


    for (unsigned int i = 0; i < this->data.ranges_count; i++)
    {
      this->data.ranges[i] = (float)this->iface->data->ranges[i];
      this->data.voltages[i]= 0;
    }

    this->driver->Publish(this->device_addr,
        PLAYER_MSGTYPE_DATA,
        PLAYER_IR_DATA_RANGES,
        (void*)&this->data, sizeof(this->data), &this->datatime);
  }

  this->iface->Unlock();
  */
}

/////////////////////////////////////////////////
void IRInterface::Subscribe()
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
    std::cout << "Error Subscribing to Gazebo IR Interface\n"
      << e << "\n";
    // gzthrow(stream.str());
    exit(0);
  }
  */
}

/////////////////////////////////////////////////
void IRInterface::Unsubscribe()
{
  /*
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->iface->Close();
  */
}
